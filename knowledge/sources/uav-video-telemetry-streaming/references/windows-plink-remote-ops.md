# Windows 主机 → 机载 Linux 的 plink/pscp 非交互运维

> 本文解决：从 Windows PowerShell 上以**密码认证**无人值守地操作机载 Jetson，
> 包括传文件、跑一次性探测脚本、托管 systemd user service。
> 适用于「指挥官模式」下用户把执行权交给 agent 的场景。

## 基础调用形态

```powershell
$PLINK = "C:\Program Files\PuTTY\plink.exe"
$PSCP  = "C:\Program Files\PuTTY\pscp.exe"
$HK    = "SHA256:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"   # 从首次连接的报错里抄
$TGT   = "fouruav@<NET_WLAN_E>.131"
```

**必须同时给 `-hostkey`**，否则 batch 模式直接放弃：

```powershell
cmd /c "`"$PLINK`" -ssh -batch -hostkey $HK -pw <SSH_PASSWORD> -m C:\tmp\probe.sh $TGT 2>&1"
```

首次连接会打印 `The server's ssh-ed25519 key fingerprint is: SHA256:...`，
把这个值抄下来填进 `-hostkey`（比 `-o StrictHostKeyChecking=no` 更干净）。

## `-m` 脚本文件的两条硬规矩

1. **必须 LF 换行，不能 CRLF。** 否则远端解析出错（可能表现为 `cat: 不适用的选项 -- R` 之类莫名其妙）。
2. **纯 ASCII**，不要写 `&`、中文、特殊符号。

PowerShell 侧转换写法：

```powershell
$s = (Get-Content "C:\tmp\probe.sh" -Raw) -replace "`r`n","`n"
[System.IO.File]::WriteAllText("C:\tmp\probe_lf.sh", $s)
```

## 传文件：用 pscp，别用 `cat >` 重定向

`plink ... "cat > /tmp/x.py" -RedirectStandardInput x.py`
在 PowerShell 下会被干扰，**实测会写出 0 字节文件**（`md5sum` = `d41d8cd9...`，即空文件）。

正确做法：

```powershell
cmd /c "`"$PSCP`" -batch -hostkey $HK -pw <SSH_PASSWORD> local.py $TGT`:/tmp/remote.py 2>&1"
# 传完必须验证
cmd /c "`"$PLINK`" -ssh -batch -hostkey $HK -pw <SSH_PASSWORD> $TGT `"ls -l /tmp/remote.py; head -3 /tmp/remote.py`" 2>&1"
```

## 后台进程：plink 下 `nohup ... &` 会挂住并返回 128

```bash
nohup python3 app.py > /tmp/app.log 2>&1 &      # ← plink 下 exit 128，且进程可能没起来
```

`setsid` / `disown` 也只是勉强。**可靠做法是交给 systemd user service：**

```bash
systemctl --user daemon-reload
systemctl --user enable mysvc.service
systemctl --user start  mysvc.service     # ← 单独 start，不要用 --now
```

**坑：`systemctl --user enable --now` 在 plink 下会返回 exit 128**（服务其实可能起来了）。
拆成 `enable` + `start` 两步，每步都 `echo EXIT=$?` 单独确认。

## plink 会话里 `systemctl --user` 能不能用

能。`XDG_RUNTIME_DIR=/run/user/1000` 与 `DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus`
已经被 sshd 注入。诊断时先打印这两个变量：

```bash
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-UNSET}"
echo "DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-UNSET}"
ls -ld /run/user/1000
```

若为 UNSET 再考虑 `export XDG_RUNTIME_DIR=/run/user/$(id -u)`。

## sudo 在 plink 下

`echo 1 | sudo -S ...` 在本 Hermes 环境会被策略拦截。两种替代：

- **需要用户亲手执行**（记忆里已确认：`echo pw|sudo -S` 一律交用户）
- 或改用**不需要 sudo** 的路径：user service、用户可写目录、`chmod` 过的设备

`fuser`、`tcpdump`、`strace` 常常没装，且装它们要 sudo → **优先用零依赖替代：**

| 想做的事 | 替代命令 |
|---|---|
| 谁在听某端口 | `ss -lntup \| grep :PORT` |
| 某进程环境变量 | `sudo tr '\0' '
' < /proc/PID/environ`（要 sudo）或 `ps eww -p PID` |
| 端口/连接是否建立 | `ss -tn "( sport = :PORT )"` |
| 抓 RTSP 原始交互 | `printf 'OPTIONS ...\r
\r
' \| nc ip port` |
| 看进程占哪个设备 | 直接看进程本身，或用 `lsof`（可能也没装） |

## 日志：`journalctl --user` 可能整个不可用

现象：`No journal files were found.` —— 用户级 journald 没落盘。
此时**不要在这上面耗时间**，改从 systemd 的 stdout 捕获：

```bash
systemctl --user status mysvc.service --no-pager | head -20   # 含最近几行日志
```

或者 **停掉服务、前台手动跑一次**，日志直接打在终端上，Ctrl+C 时的 traceback
往往直接暴露阻塞位置（这是本文最有效的调试手段）。
跑完记得 `systemctl --user start mysvc.service` 恢复。

## 相关技能
- `uav-video-telemetry-streaming`：图传/数传搭建（本文是它的运维底座）
- `uav-ros-sensor-diagnostics`：飞控/LiDAR 排查
- `ssh-remote-linux-ops`、`windows-ssh`、`windows-ssh-password-auth`：其他 SSH 场景
