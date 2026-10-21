# 搭建收尾：git 推送工作站 + Windows 侧验证 + ament_python 包骨架

本文件对应"环境装好后，建 ROS2 工作站、推到私有 GitHub、以及无 ROS 环境下如何验证"这一收尾段。

## 1. git 推送到私有 GitHub 仓库（HTTPS + PAT，不是密码）

**关键**：GitHub **已废弃登录密码认证**。`git push` 提示 `Username`/`Password` 时，`Password` 一栏必须填 **Personal Access Token（PAT）**，不是你的 GitHub 账号密码。若填了登录密码，会报：
```
remote: Invalid username or token. Password authentication is not supported for Git operations.
fatal: Authentication failed for 'https://github.com/<user>/<repo>.git'
```

**生成 PAT**：`https://github.com/settings/tokens/new` → Name 随意 → ⚠️ **勾 `repo` scope**（整套 repo 权限，否则私有仓库 push 不通过）→ Generate → 复制 `ghp_...`（只显示一次）。

**让 git 记住，免每次输入**：
```bash
git config --global credential.helper store     # 明文存 ~/.git-credentials
# 介意明文可用 cache：git config --global credential.helper 'cache --timeout=3600'
git config --global user.name "<GitHub用户名>"  # 提交作者，WSL 首次必须配
git config --global user.email "<GitHub绑定邮箱>"
```

**私有仓库首次 push 前诊断**：
- 探测仓库是否私有 + 账号是否可用：`GIT_TERMINAL_PROMPT=0 git ls-remote <url>`（私有且无凭据 → 报 `could not read Username`；能列 refs = 可匿名访问/公开）。注意 `ls-remote` 在 WSL 里非交互会因缺凭据而 128，用它判断"是否私有"即可。

**push 被拒（远程非空）**——常见于 GitHub 建库时勾了 "Add README/.gitignore" 自动初始化文件，与本地历史不共享：
```
! [rejected] main -> main (fetch first)
Updates were rejected because the remote contains work that you do not have locally.
```
- 若远程只有 GitHub 自动生成的占位文件、你要用本地整套覆盖 → `git push -u origin main --force`（⚠️ 确认远程无重要内容）。
- 若远程有你想保留的内容 → `git pull --rebase origin main`（README 冲突则选保留的一份）再 push。

## 2. 无 ROS 环境下如何验证一个 ament_python 包（在 Windows 主机侧）

agent 若运行在 Windows 主机上没有 ROS2/colcon 环境，**无法对 ROS 包做构建级验证**。能做的是**静态 ad-hoc 验证**（不代表构建通过）：

1. **Python 语法**：`py_compile.compile(path, doraise=True)` —— 只编译为字节码、不执行，因此即使 `import rclpy` 也不报错（只要语法对）。对 `.py` 源码做语法检查有效。
2. **package.xml 结构**：`xml.etree.ElementTree.parse(...)` 确认 well-formed、`name=<pkg>`、`export/build_type` 正确。
3. **真正的构建验证**必须移到 WSL：先 `sudo apt install -y python3-colcon-common-extensions`（Jazzy desktop 不总带 colcon），再 `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install`，最后 `install/setup.bash && ros2 pkg list | grep <pkg>`。

## 3. ament_python 最小功能包骨架（能 colcon build 通过的最低结构）

```
ros2_ws/
└── src/<pkg>/
    ├── package.xml          # <build_type>ament_python</build_type>; exec_depend rclpy/sensor_msgs/geometry_msgs
    ├── setup.py             # find_packages() + data_files(resource_index/packages + share/<pkg> + launch) + entry_points console_scripts
    ├── setup.cfg            # [develop]/[install] 两段 script_dir, install_scripts
    ├── resource/<pkg>       # 空文件（ament index 需要）
    └── <pkg>/__init__.py + <模块>.py   # 节点模块
```

要点：
- `setup.py` 的 `entry_points:'console_scripts':['<节点名>=<pkg>.<模块>:main']` 让 `ros2 run` 能找到。
- `resource/<name>` 空文件 + `package.xml` 的 `export` + `data_files` 三处对齐，否则 colcon 报 resource_index 缺失。
- `src/` 空会不被 git 追踪，用 `touch src/.gitkeep` 占位。
- 节点骨架：`rclpy.init()` → `create_subscription`/`create_publisher` → `rclpy.spin(node)` → `shutdown`。示例见本仓库 `exploration/frontier_explorer.py`（Subscribe LaserScan → 发 Twist，占位跑通链路）。
