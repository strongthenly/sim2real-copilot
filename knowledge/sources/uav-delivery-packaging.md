---
name: uav-delivery-packaging
description: 无人机机载代码的交付打包——防源码泄露的"二进制-only"方案（只给 devel/lib 编译产物 + 运行配置，零 .py/.git/日志）与"干净快照"方案（git archive 脱敏）。含二进制新鲜度验证（源码 mtime vs 二进制 mtime，切勿用 git 提交时间判新旧）、敏感内容自检、pscp 传回 Windows。客户/合作方要"最新仓库压缩包"时用。
---

# UAV 交付打包（防源码泄露）

## When to use
- 有人（客户/合作方/验收方）要求提供"最新仓库的压缩包"，但需防止源码泄露
- 需要确认机载二进制是否为最新（编译产物 vs 源码）
- 需要从机载打包脱敏交付物并传回 Windows

### ⛔ 先分清是**对外交付**还是**自用部署**（两者流程完全不同）
| 场景 | 走哪里 |
|---|---|
| **对外交付**（客户/合作方要压缩包、要防源码泄露）| ← **本技能** |
| **自用部署**（把仿真里跑通的层搬到自己飞机上）| **不要用本技能** —— 全源码 + 一键脚本，走 `uav-super-navigation/references/sim-to-real-fuel-deployment.md`（含 sim→real 的适配清单、实机安全差异、验证阶梯、**上机前强制预检 `MID360_Fly.yaml` 的 `dense_map_en` 有没有被改过**）|
判据：用户说"**导出一版可以用在实机上面**" ⇒ 自用部署 ✗；说"给客户/合作方/验收方" ⇒ 本技能 ✓
**误判代价**：按防泄露流程去打包自用部署，会删掉自己需要的源码、多绕一大圈。

## 关键决策：先问交付场景（三选一）
| 场景 | 给什么 | 手段 |
|---|---|---|
| A 验收/存档/演示 | 干净快照（当前 HEAD） | `git archive` + `--exclude` 敏感文件 |
| B 部署但防源码泄露 | **编译好的二进制 + 运行配置** | 拷贝 devel/lib ELF + config/launch/rviz/航迹 |
| C 只给文档 | README + 架构说明 + 截图 | 最安全 |

用户已拍板过方案 B（2026-08-25）→ 后又演进：B+（全量 src+pyc）→ **方案 C（最终）：NX_code Cython .so 保密 + 其他工程开源**。动手前先问当前是哪个方案，别默认。每次拍板都问清"保密哪些、开源哪些、内部文档哪些不发布"。

## Step 1 敏感面侦察（打包前必做）
```bash
du -sh . .git
find . -path ./.git -prune -o -type f \( -name "*.bak*" -o -name "*.log" -o -name "*.pem" -o -name "*.key" -o -name "*password*" -o -name "*secret*" -o -name "*token*" \) -print
grep -rniE "password|192\.168\.|10\.48\.|git@github|ssh://" --include="*.yaml" --include="*.sh" --include="*.launch" --include="*.py" .
git rev-list --count HEAD   # 泄露面：历史提交数
```
红色泄露点：`.git/`（全部历史+调参过程）、`log/`（真实飞行数据）、`*.bak*`、launch 内地面站 IP、`.github/workflows`（GITHUB_TOKEN）。

## Step 2 二进制新鲜度验证（核心坑，2026-08-25 实测踩过）
**判定二进制是否最新：比较"源码文件 mtime vs 二进制 mtime"，绝不能拿 git 提交时间！**
```bash
# 正确：找该目标对应的源码文件，看 8-14 后是否真的改过
find ~/catkin_ws/src/SUPER/super_planner \( -name "*.cpp" -o -name "*.h" \) -newermt "2026-08-14 18:01" -type f
stat -c '%y' ~/catkin_ws/devel/lib/super_planner/fsm_node
```
踩坑实录：git 提交 2fbadee 时间是 8-19，但那次只改了 `super_to_px4_bridge.cpp`；fsm_node 源码 8-14 后没动 → fsm_node 二进制 8-14 就是最新的。误用 git 提交时间会误判"落后 11 天"，白白触发一次全量编译。
另注意：catkin 增量日志里 "Built target X" 只表示 make 检查过该目标，**不表示重新编译**（无变更时同样打印 Built）。

## Step 3 打包（方案 B 结构）
```
pkg/
├── bin/             # devel/lib/{super_planner,misssion_planner,task_run_scripts}/ 下所有 ELF
├── config/          # *.yaml
├── launch/          # 含 odin/ fastlivo2/ 子目录
├── rviz/
└── data_guidepoints/
```
一键脚本：`scripts/package_binary_only.sh`（改 SRC/PKG 变量即可，自带自检）。

## Step 4 自检清单（交付前必须全过）
1. 包内 `.py` 数 = 0（源码零泄露）
2. 包内 `.bak` / `.log` / `.git` 数 = 0
3. 文本文件 grep IP/密码/token = 0（机载 IP 泄露会暴露内网）
4. ELF magic：前 4 字节 `7f 45 4c 46`（= 127,69,76,70）。注意 `tar -xOf` 管道读二进制会误报退出码，用 `[IO.File]::ReadAllBytes` 或远端 `file` 命令
5. **aarch64 二进制只能在 ARM/Jetson 跑**——交付前告知对方目标平台，x86 跑不了
6. Python 节点（global_path_publisher、odom_to_mavros、NX_code 按钮服务等）是源码，**进不了二进制包**——对方拿到的包跑不了完整系统，这是防泄露的代价，要明确告知

## 方案 B+（2026-08-25 拍板：全量 src + .py→.pyc + 带 devel，比纯 B 更能跑）
用户需求演进：不只是 task_run_scripts，而是**整个 ~/catkin_ws/src 全量打包**，且接受 `.py 编译成 .pyc`（字节码）以保留可运行性。此方案下 C++ 源码（.cpp/.h）**删除**，只留编译产物；**devel/ 必须一起打包**（否则对方没有 setup.bash/消息库/.so，`source devel/setup.bash` 都做不了，拿到就是废包）。

### 防泄露处理清单（每个包统一）
| 内容 | 处理 |
|---|---|
| `.git/` + 子模块 .git 指针文件 | 删（find -name .git -exec rm -rf / -delete 双管齐下） |
| `.cpp/.h/.hpp/.cc/.cxx/.c` | 删（本次实测删了 741 个） |
| `.py` | `python3 -m py_compile` → `__pycache__/xx.cpython-38.pyc` 复制为**同目录 `xx.pyc`** → 删 .py |
| `launch` 里 `type="xx.py"` | `sed -i 's/type="\([^"]*\)\.py"/type="\1.pyc"/g'` 批量改（本次改了 3 个） |
| `devel/` 里的 .py（消息库等） | 同样编译；**保留 `__pycache__`**（Python import 机制靠它加载字节码），与 src 脚本的"同目录 xx.pyc"不同 |
| `log/`、`*.log`、`*.bak*` | 删 |
| `package.xml`/`CMakeLists.txt`/launch/config/rviz/msg/srv/data | 保留（接口/配置非实现） |

### 体积控制（必做！否则 tar 卡死）
全量 src 有巨型数据文件：本次发现 `FAST_LIO/PCD/scans.pcd` 1.6GB、`FAST-LIVO2/Log/pcd/*.pcd` 221MB+42MB×3、sim 的 random_map pcd。**打包前先 `du -sh ~/catkin_ws/src/*/` + `find src -type f -printf '%s %p
' | sort -rn | head` 查大头**，排除 `*.pcd/*.bag/*.ply/*.bin/*.mp4`（既占体积又泄露真实飞行数据）。src+devel 全量可达 3.4G，tar.gz 压缩几十分钟且易被打断；排除数据文件后可到数百 MB。

### strip 二进制（防逆向泄露面）
devel/lib 的 ELF 是 `not stripped, with debug_info`——debug_info 含**源码路径、全部函数名、变量名、行号映射**（如 `/home/fouruav/catkin_ws/src/SUPER/...`），对方用 IDA/Ghidra/strings 可提取 ≈ 半个源码。交付前 `strip --strip-unneeded <bin>` 删 debug_info+符号表，体积还能减半（fsm_node 215MB→~50MB）。

## 方案 C（2026-08-25 最终拍板）：NX_code Cython 保密 + 其他工程开源
用户原话："我要求只编译NX_code工程，其他工程开源给客户"、"其中是否有一些文档是内部文档，内部文档不发布"。这是比 B/B+ 更细的分级：**自研服务保密（Cython .so），算法/驱动/配置全开源**。

### NX_code Cython 编译（现成脚本，勿重写）
```bash
cd ~/catkin_ws/src/task_run_scripts/scripts/NX_code
bash build_secure.sh   # 前置: python3 -c "import Cython" + gcc
```
- 产出 `deploy/`：6 个 `.cpython-38-aarch64-linux-gnu.so`（nx_command_server/nx_file_transfer/rviz_full_stream/nx_touch_server/odom_to_mavros/nx_paths）+ `*_launcher.py`（薄启动器，仅 import .so，**无业务逻辑**，随包可接受）+ `*2.py` 兼容包装器 + 资源文件（yaml/sh/desktop）
- 源目录 NX_code/ 保持完整源码不动（脚本只动 deploy 副本）
- 打包时用 deploy/ 整体替换源码 NX_code 目录（`rm -rf 目标; cp -a deploy/ 目标`）
- **编译脚本 setup.py 在 NX_code 目录内**（先 `ls setup.py` 确认存在），gcc/Cython 缺失先装

### 隐私扫描（开源交付前必做，本会话实测发现大量泄露）
```bash
grep -rnoE "(192\.168\.|10\.|172\.(1[6-9]|2[0-9]|3[01])\.|169\.254\.)" <pkg>   # IP
grep -rl "/home/fouruav" <pkg>                                                # 绝对路径(本会话103文件!)
grep -rnoE "(fouruav|jihaofly|jihaoxiang)" <pkg>                              # 用户名(108文件)
grep -rnoE "(github_pat_|ghp_|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY)" <pkg>      # token/密钥(应为0)
```
- **`/home/fouruav` 硬编码**：业务脚本（User_define_commands.yaml/start_qgc_servers.sh/*.desktop）+ devel 编译产物（cmake/pc/setup/_setup_util.py，**无法清除**）+ ELF 二进制内嵌（strings 可见，无法清除，不影响运行）
- **客户部署影响**：路径硬编码 → 客户建同名用户 fouruav 最稳（开箱即用）；换路径要逐个 sed 且易配错。二进制内嵌路径只泄露账号名，风险可接受
- **现有 shipment_privacy_cleaner.sh 只清机器痕迹**（历史/缓存/日志/凭据），**不清包内容**——别指望它
- 判据：token/密钥/SSH 私钥 = 0 是最低线；IP/路径泄露严重度取决于客户关系

### 内部文档识别（开源交付时排除，列清单让用户确认）
- **排除**：`AGENTS.md`（开发约定）、`debug-flight-plan.md`（🔴 调参方案，敏感）、`guide_Readme.md`、`troubleshooting_Readme.md`、`DEPLOY_CI.md`、`obstacle_warning.md`
- **保留**：各仓库 README.md、LICENSE/CHANGELOG、CMakeLists.txt/package.xml（构建必需）
- **边界项要问用户**：SUPER/README.md（项目主页）、`data_guidepoints/guidepoints_2026*.txt`（**真实飞行位置数据**，泄露场地）、odin_ros_driver 第三方 SDK（确认分发权）

## 绑定链验证（打包/发布前必做，避免交付无关二进制）
用户 2026-08-25 要求确认"SUPER 绑定在智能航线任务上"。逐层 grep 验证 launch 链：
```
UserCmd1(正式版) → odin_real_flight.launch → include real_flight.launch
  → fsm_node / super_to_px4_bridge / waypoint_mission（都是 SUPER 编译产物）
```
命令：`git show HEAD:User_define_commands.yaml | head`、`cat launch/odin/odin_real_flight.launch`、`grep -n "pkg=\|type=" launch/real_flight.launch`、`grep -n "guide_path" SUPER/super_planner/src/super_core/super_planner.cpp`。另注意：**User_define_commands.yaml 工作区可能是临时测试版**（如 UserCmd1 只剩 `CP_DIST=-1`，注释"原航线任务功能暂时移除"），打包/发布前要问用户是否 `git checkout -- User_define_commands.yaml` 恢复正式版（先 cp 备份）。

## GitHub Release 发布（替代打包：用户 2026-08-25 最终拍板"基于最新 main 发版本，从 GitHub 直接下载"）
发布前检查（一次脚本全查）：
```bash
git status -sb                                    # 分支与 origin 同步
git remote -v                                     # origin URL
ls ~/.ssh/ && timeout 8 ssh -T git@github.com     # SSH key + 认证（期望 "Hi <user>!"）
git tag | tail                                    # 已有 tag
```
关键提醒：Release 源码包 = git archive 当前提交，**含全部源码**——必须先确认仓库 **private** 才安全（public 发布=全世界可见源码，与防泄露目标矛盾）。功能齐全验证清单：核心文件 12 项齐全（launch/config/航迹/发布器/rviz）+ 编译产物 7 个 ELF + 子模块指针一致。发布 = `git tag vX.Y.Z` + `git push origin main --tags` + gh/API 建 Release。

### Release API 实操（2026-08-25 实测，无 gh CLI 时 curl 全流程）
- **发布到 catkin_ws 总仓库**（`<ORG>/<PRIVATE_WS_REPO>`，私有，子模块总仓），**不是子仓库**（task_run_scripts）——用户拍板纠正过
- **版本标识带传感器名**：`v1.0.0-odin`（tag/Release 标题/asset 文件名三处一致）
- **本地 master 落后远端时只 push tag 不 push branch**：`git push origin v1.0.0-odin`（tag 不受 fast-forward 限制；branch 会被拒）——先本地 `git tag -a` 再 push
- 创建 Release：`curl -X POST https://api.github.com/repos/{o}/{r}/releases -H "Authorization: Bearer $TOKEN" -d '{"tag_name":"v1.0.0-odin","name":"...","body":"...","draft":false}'` → 解析 `id`
- 上传 asset：`curl -X POST "https://uploads.github.com/repos/{o}/{r}/releases/{id}/assets?name=catkin_ws_odin_latest_v1.0.0.tar.gz" -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/gzip" --data-binary @file`（200MB 约 20-30s）
- **误发到错误仓库**：`DELETE /repos/{o}/{r}/releases/{id}`（204）+ `DELETE /repos/{o}/{r}/git/refs/tags/{tag}`（204），验证 `GET /repos/{o}/{r}/releases/tags/{tag}` 返回 Not Found
- **PAT 最小权限**：fine-grained token，只选目标仓库 + `Contents: Read and write` + 7 天过期（classic token 的 `repo` 覆盖所有仓库，范围大）；**用完必须撤销**（token 会出现在对话/脚本里，留在外面能访问私有仓库）

## Step 5 传输回 Windows
pscp（必须 cmd /c 包装，禁止裸 `&` 调用，见 windows-ssh 技能）：
```powershell
cmd /c "`"C:\Program Files\PuTTY\pscp.exe`" -scp -batch -hostkey `"SHA256:...`" -pw <SSH_PASSWORD> `"fouruav@IP:/tmp/pkg.tar.gz`" `"C:\Users\...\pkg.tar.gz`""
```
下载后本地解压复检（清单/敏感词/大小一致），再交付。

## Pitfalls
- **`&` 出现在 plink 远程命令字符串内也会被 Hermes 终端工具拒绝**（"Foreground command uses '&' backgrounding"），即使整个 cmd /c 已加引号——远程要 nohup 后台执行时，把命令写进脚本文件用 `plink -m` 传（LF 转换），别内联 `&`。
- 子模块的 `.git` 是**文件**不是目录，`[ -d .git ]` 判仓库会误报"不是 git 仓库"——用 `git -C "$d" rev-parse --git-dir`。
- pkill -f 在 plink -m 脚本里会匹配脚本自身 argv（bash -c 全文），exit 128 静默被杀——详见 windows-ssh-automation 技能。
- 编译/文件时间用 `stat -c '%y'`，别信 `ls` 的日期格式（跨 locale 月份解析坑）。
- 打包前先问用户：要 commit/push 还是只留本地修改（用户 2026-08-25 明确"别提交，本地修改"——改完保持 git status 为 M 即可）。

## Support files
- scripts/package_binary_only.sh — 一键构建二进制-only 交付包（拷贝 ELF+配置 → 自检 → tar.gz）
- scripts/package_src_pyc.sh — 方案 B+ 一键打包（全量 src + devel：删 .git/C++ 源码、.py→.pyc、launch type 改 .pyc、体积侦察、可选 strip）
- scripts/privacy_scan.sh — 发货前隐私扫描（IP/绝对路径/用户名/token/密钥文件/残留，见"方案 C"节）
