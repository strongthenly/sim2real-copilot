# Windows 主机 → Linux 远端 的密码 SSH 技巧

场景：Windows 11 主机（PowerShell 5.1）SSH 到 Ubuntu/Jetson（密码认证，无密钥）。本会话踩坑记录。

## 为什么默认 OpenSSH 不行
- `ssh user@host` 在 PTY 后台模式反复崩溃，exit code `0xC000013A`（STATUS_CONTROL_C_EXIT），无输出
- Windows OpenSSH **不从 stdin 读密码**，交互式密码提示在自动化环境无法处理
- `SSH_ASKPASS` + .bat 脚本在 Windows 上不生效（需要 GUI 程序返回密码）
- plink / sshpass 默认未安装，`python` 是商店占位符不可用

## 正解：plink.exe
1. 下载：`curl.exe -L -o "$env:TEMP\plink.exe" "https://the.earth.li/~sgtatham/putty/latest/w64/plink.exe"`（~1MB）
2. 获取 host key 指纹：先跑一次 `ssh -v user@host`，从输出抓 `Host 'xxx' is known and matches the ED25519 host key` 之前的 `Server host key: ssh-ed25519 SHA256:xxxx`
3. 非交互登录执行单条命令：
   ```powershell
   & "$env:TEMP\plink.exe" -ssh -batch -hostkey "SHA256:xxxx" -pw <密码> user@host "hostname; whoami; uname -a"
   ```
   - `-batch` 禁用交互提示；`-hostkey` 跳过首次确认；`-pw` 直接给密码
   - 单引号字符串内含 `$` 等需注意 PowerShell 转义；含引号复杂命令用 base64 方案

## 多行/复杂脚本传输（base64 方案）
PowerShell 管道给 plink 会触发 "uses '&' backgrounding" 误判或引号地狱。可靠做法：
1. 本地用 write_file 写脚本 `diag.sh`
2. PowerShell 转 base64：
   ```powershell
   $content = Get-Content "C:\...\diag.sh" -Raw
   $b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($content))
   cmd /c "`"$env:TEMP\plink.exe`" -ssh -batch -hostkey `"SHA256:xxxx`" -pw <密码> user@host `"echo $b64 | base64 -d | bash`""
   ```
3. 注意：脚本内不要用 heredoc（会被 base64 管道破坏），用 `python3 -c "..."` 单行替代

## plink -m 脚本文件方式（推荐，比 base64 更简单）
多行命令直接写成本地 .sh 文件，plink 用 `-m` 参数执行，无需 base64 管道/引号嵌套：
```powershell
cmd /c "`"$env:TEMP\plink.exe`" -ssh -batch -hostkey `"SHA256:xxxx`" -pw <密码> user@host -m `"C:\path\script.sh`""
```
- 脚本内容为 bash（`export LANG=C.UTF-8` 防中文乱码），write_file 写好直接传
- 比 cmd /c 内联引号嵌套、比 base64 管道都干净，实测稳定（搜索远端文件/多段 cat 都可用）
- 注意：PowerShell 里 `& "$env:TEMP\plink.exe" ...` 的 `&` 调用符会被终端误判为后台化而拒绝执行，用 `cmd /c` 包装即可

## sudo 提权
plink 无 TTY，`sudo cmd` 报 "a terminal is required"。用 `echo "<密码>" | sudo -S cmd`。

## 其他要点
- 中文输出在 plink 下乱码是编码问题，内容仍可读；可在远端 `export LANG=C.UTF-8`
- 用户名拼写要仔细核对（本会话 `fouruav` 被误打成 `fouuav` 导致 Permission denied，用户回复命令本身才纠正）
- 密码错误时 plink 报 `FATAL ERROR: Configured password was not accepted`；用户名/密码都错时 `Access denied`
- 远端进程后台化：`nohup cmd > /tmp/x.log 2>&1 &`，之后用 `tail` 看日志
