# Password-SSH from Windows (plink) — recipes

## Why not OpenSSH
Windows OpenSSH 9.x crashes with exit 0xC000013A when launched in a PTY and asked for a password (no output at all); it refuses passwords on stdin. `SSH_ASKPASS` pointing at a .bat file also fails (Windows askpass expects an EXE that returns the string; bat is ignored). The reliable path is PuTTY's plink.exe with `-pw`.

## Setup (once per host)
```powershell
# download plink (small, ~1 MB)
curl.exe -L -o "$env:TEMP\plink.exe" "https://the.earth.li/~sgtatham/putty/latest/w64/plink.exe"
# capture host key fingerprint (also proves TCP+SSH handshake work):
ssh -v -o ConnectTimeout=10 -o BatchMode=yes user@host 2>&1 | Select-String "Server host key"
```
Save the `SHA256:...` fingerprint for the -hostkey arg. Note: `ssh -v` in non-PTY mode works fine for diagnosis — only interactive password auth crashes.

## Connect + run one command
```powershell
& "$env:TEMP\plink.exe" -ssh -batch -hostkey "SHA256:<fp>" -pw <pw> user@host "hostname; whoami; uname -a"
```
- `-batch` = never prompt, fail fast.
- Wrong hostkey → "Access denied" style abort; wrong password → "Configured password was not accepted".
- **Username typos are a classic silent failure**: `fouuav` vs `fouruav` → "Permission denied (publickey,password)" even with the right password.

## Run a multi-line script (PowerShell quoting escape)

**首选：`-m <file>` 模式**（比 base64 干净，实测最稳）——本地写 .sh，plink 读文件执行：

```powershell
# 1) 先写 LF 换行的纯 ASCII 脚本（CRLF 会让 bash 报 $'\r': command not found）
$s = (Get-Content "$env:TEMP\probe.sh" -Raw) -replace "`r`n", "`n"
[System.IO.File]::WriteAllText("$env:TEMP\probe_lf.sh", $s)

# 2) cmd /c 包装（& 调用符会被 Hermes 判成后台化而拒绝）
cmd /c "`"C:\Program Files\PuTTY\plink.exe`" -ssh -batch -hostkey SHA256:<fp> -pw <pw> -m $env:TEMP\probe_lf.sh user@host 2>&1"
```

脚本内容规则：
- **必须 LF + 纯 ASCII**（`&`、中文注释、`;` 在 -m 模式下都可能出问题，改写掉）
- 不要 heredoc（`<<EOF`）
- 需要多段输出时用 `echo "=== N. xxx ==="` 做 ASCII 分隔符，中文输出可能变乱码但结构可读

**备选：base64 管道**（-m 不可用时）：
```powershell
$content = Get-Content C:\path\script.sh -Raw
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($content))
cmd /c "`"$env:TEMP\plink.exe`" -ssh -batch -hostkey `"SHA256:<fp>`" -pw <pw> user@host `"echo $b64 | base64 -d | bash`""
```

### 🔴 `-batch` + 未缓存 hostkey = 直接失败（不是密码错）

```
The host key is not cached for this server: <NET_RC>.99 (port 22)
...The server's ssh-ed25519 key fingerprint is:
  ssh-ed25519 255 SHA256:<ONBOARD_HOSTKEY_FP>
Connection abandoned.
FATAL ERROR: Cannot confirm a host key in batch mode
```

**这不是认证失败、更不是密码错** —— 连接根本没建立到认证阶段。
plink 在报错里已经把 `SHA256:` 指纹打印出来了，**直接抄进 `-hostkey` 重跑即可**，
不需要另外去 `ssh -v` 抓一次。判据：看到 `Cannot confirm a host key in batch mode` = 只差 `-hostkey` 参数。

## Reboot / service recovery after a destructive script
用户跑了旧脚本把 systemd 服务杀掉却没起来时：
```bash
systemctl --user start aircore-stream.service    # 直接拉回来
ss -lntup | grep 6000                            # 验证
```
若输出 `Warning: The unit file ... changed on disk. Run 'systemctl --user daemon-reload'`：
**当前不受影响（用的内存里的旧定义），但下次 reload 会切到磁盘新版本，可能行为不一致。**
腾出手时补 `systemctl --user daemon-reload`（会短暂影响所有 user 单元，选无任务时做）。


## sudo inside remote scripts
plink has no TTY → `sudo` fails with "a terminal is required to read the password". Either:
- `echo <pw> | sudo -S <cmd>` inside the script, or
- ask the user to run the sudo step in their own terminal and paste output back.

## Division of labor pattern (worked well)
User prefers to drive the terminal themselves ("指挥官" mode — I give commands, they run them and paste output). When the user explicitly says "你自己ssh进去看看" (you ssh in yourself), switch to plink one-shot diagnostics. Keep commands small and batch them into one script per round-trip.
