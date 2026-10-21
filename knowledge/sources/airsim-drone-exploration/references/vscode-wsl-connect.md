# 用 VS Code 连接 WSL 发行版（Remote-WSL）

> 触发：用户说“我想用 VS Code 连我现在的 WSL” / “code . 打不开” / “vscode-server 装不上”。

## 组成（本机实测）
- **Windows 侧**：VS Code（本机 **1.137.0**，装在 `C:\Users\<USER>\AppData\Local\Programs\Microsoft VS Code\`）
  + 扩展 **`ms-vscode-remote.remote-wsl`**（本机 0.104.3）
- **WSL 侧**：`~/.vscode-server`（首次连接自动安装；本机 **654 MB**）
- WSL 里的 `code` 命令来自 **Windows PATH**（`/mnt/c/.../Microsoft VS Code/bin/code`；WSL 默认 `appendWindowsPath=true`，`/etc/wsl.conf` 没关就有）

## 步骤
1. **确认扩展已装**——用**目录**查，不要用 CLI：
   ```powershell
   Get-ChildItem "$env:USERPROFILE\.vscode\extensions" -Directory |
     Where-Object { $_.Name -match 'wsl|remote' } | Select-Object -ExpandProperty Name
   ```
2. **首次安装 server（必须走代理）**：
   ```bash
   export https_proxy=http://127.0.0.1:7899 http_proxy=http://127.0.0.1:7899
   export HTTPS_PROXY=$https_proxy HTTP_PROXY=$http_proxy
   code --version          # 这条就会触发 Installing VS Code Server for Linux x64 → Downloading
   ```
   成功标志（stdout 末尾）：`Compatibility check successful (0)` + 版本号 + commit hash；
   目录：`~/.vscode-server/bin/<commit>/`（`du -sh ~/.vscode-server` 约 650MB）。
3. **连接**（三种都行）：
   - 在 WSL 的目标目录里：`code .`
   - VS Code 左下角绿色 `><` → **Connect to WSL** → 选发行版
   - `Ctrl+Shift+P` → `WSL: Connect to WSL`
4. **验证**：
   - **Windows 窗口标题 = `<文件夹> [WSL: <发行版>] - Visual Studio Code`**
   - WSL 侧 `pgrep -af vscode-server` 有 `code-server ... --host=127.0.0.1`（还有 `ms-vscode.remote-wsl-*/scripts/wslServer.sh`）
   - `Get-Process Code | Where MainWindowTitle -ne ''` 能看到该标题

## 坑
- **`code --list-extensions` / `code --version` 会拉起 GUI 然后挂住**（本会话在 Hermes 里等到 180s 超时）。
  → 查扩展一律**看 `.vscode\extensions` 目录**；确实要跑 CLI 就套 `timeout`，并预期它可能不返回。
- **首次 server 下载走不动 = 没走代理**（不是 VS Code 坏）。设 `https_proxy`/`http_proxy` 后**重跑 `code --version`** 即续传；装完后再连**不会**再下载（秒连）。
- 别名/多发行版：`code .` 连的是**当前所在的发行版**；要从别处连指定发行版用命令面板 `WSL: Connect to WSL using Distro...`。
- VS Code 的**集成终端默认就在该发行版里**（`Ctrl+``）→ 可直接跑 `bash ~/twin_livo_fast.sh`、`rostopic`、`rosrun`，不必再开 Windows Terminal。
- 卸载/迁移：删 `~/.vscode-server` 即回退到“未连接”状态（下次会重装）。
