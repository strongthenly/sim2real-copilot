#!/bin/bash
# ============================================================
#  ROS 桥冒烟测试骨架（canonical test；配 Makefile 的 test: 目标）
#
#  用法: bash <pkg>/test/smoke_test.sh        （在 WSL 内执行）
#  设计要点（照抄，别简化掉）:
#   1. 用【独立 master + 独立端口】，不干扰在跑的链路
#   2. 分【两个实例】—— 带阶段的桥一旦进入后段状态就永不回退，
#      单实例测不全「前段行为 + 后段行为」
#   3. 断言前先 sleep 等异步状态生效，再 clear() 计数
#   4. 用 Python 发消息（绕开 rostopic pub 的 YAML/引号坑）
#   5. 只读部分（诊断类脚本）必须断言"不含 publish/set_mode/arming"
# ============================================================
source /opt/ros/noetic/setup.bash 2>/dev/null      # 换成你的 distro setup
source ~/WS/devel/setup.bash 2>/dev/null           # 换成你的工作空间
PKG=my_bridge                                       # ← 改成你的包名
D=~/WS/src/$PKG                                     # ← 改成你的包路径
FAIL=0
ok(){ echo "  PASS $1"; }
no(){ echo "  FAIL $1"; FAIL=1; }

# ---------- A) 静态 ----------
echo "===== A) 静态 ====="
python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('$D/scripts/*.py')]" \
  && ok "scripts py 语法" || no "scripts py 语法"
python3 -c "import xml.etree.ElementTree as E;E.parse('$D/launch/xx.launch')" \
  && ok "launch XML" || no "launch XML"
for s in start stop; do
  bash -n "$D/scripts/$s.sh" && ok "$s.sh 语法" || no "$s.sh 语法"
done
# 关键不变量（按你的设计改断言对象）
grep -q "if not self.aligned():" "$D/scripts/*ctrl*.py" && ok "含未对齐防撞保险" || no "缺防撞保险"
grep -qE 'init_z" +default="\$\(arg seed_z\)' "$D/launch/xx.launch" && ok "init_z 继承 seed_z" || no "init_z 未继承"

# ---------- 启动/清理助手 ----------
master(){ export ROS_MASTER_URI=http://127.0.0.1:$1
  pkill -f "rosmaster --core -p $1" 2>/dev/null; sleep 1
  setsid rosmaster --core -p $1 >/dev/null 2>&1 </dev/null & sleep 3; }
cleanup(){ pkill -f "$PKG" 2>/dev/null; sleep 1; }

# ============================================================
#  实例1：测【前段行为】—— 例如 seed 悬停 + 未对齐时拒绝转发
#        关键：让后段状态【永不满足】(align_samples 设极大)
# ============================================================
echo ""
echo "===== B) 实例1（align_samples=1000，后段永不满足）====="
master 11351
setsid rosrun $PKG bridge_in.py  >/tmp/sm_in.log 2>&1 </dev/null &
setsid rosrun $PKG bridge_ctrl.py _auto_offboard:=false _seed_z:=1.5 _align_samples:=1000 \
  >/tmp/sm_c1.log 2>&1 </dev/null &
sleep 5
pgrep -f bridge_in.py   >/dev/null && ok "输入桥在跑" || no "输入桥未起"
pgrep -f bridge_ctrl.py >/dev/null && ok "控制桥在跑" || no "控制桥未起"

python3 - <<'PY' || FAIL=1
import sys, rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from quadrotor_msgs.msg import PositionCommand            # ← 换成你的 msg 包
rospy.init_node('sm1', anonymous=True, disable_signals=True)
F=[]
def ck(c,m): print("  %s %s" % ("PASS" if c else "FAIL", m)); F.append(c)

od,ps,cl,sp=[],[],[],[]
rospy.Subscriber('/odom_world',Odometry,lambda m:od.append(m))
rospy.Subscriber('/map_ros/pose',PoseStamped,lambda m:ps.append(m))
rospy.Subscriber('/map_ros/cloud',PointCloud2,lambda m:cl.append(m))
rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,lambda m:sp.append(m))
po  = rospy.Publisher('/aft_mapped_to_init',Odometry,queue_size=10)
pc2 = rospy.Publisher('/cloud_registered',PointCloud2,queue_size=10)
plp = rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=10)
pcm = rospy.Publisher('/planning/pos_cmd',PositionCommand,queue_size=10)
rospy.sleep(2)      # 等订阅建立（否则第一条会丢）

# 1) 输入桥：转发 + frame 改写
o=Odometry(); o.header.frame_id='camera_init'
o.pose.pose.position.x=1.0; o.pose.pose.orientation.w=1.0
c2=PointCloud2(); c2.header.frame_id='camera_init'
c2.height=1; c2.width=2; c2.point_step=16; c2.row_step=32; c2.data=[0]*32
for _ in range(5): po.publish(o); pc2.publish(c2); rospy.sleep(0.1)
rospy.sleep(0.5)
ck(bool(od) and od[-1].header.frame_id=='world', "输入桥 /odom_world frame→world")
ck(bool(ps) and abs(ps[-1].pose.position.x-1.0)<1e-6, "输入桥 /map_ros/pose 位姿透传")
ck(bool(cl) and cl[-1].header.frame_id=='world', "输入桥 /map_ros/cloud frame→world")

# 2) 前段：seed 悬停（此时还没发过任何外部指令）
s=PoseStamped(); s.header.frame_id='world'
s.pose.position.x=0.5; s.pose.position.y=0.6; s.pose.position.z=0.0; s.pose.orientation.w=1.0
sp.clear()
for _ in range(15): plp.publish(s); rospy.sleep(0.1)
rospy.sleep(0.5)
ck(bool(sp) and abs(sp[-1].pose.position.z-1.5)<0.02 and abs(sp[-1].pose.position.x-0.5)<0.02,
   "seed 悬停 → (0.5, 0.6, 1.5)")

# 3) 后段：发目标但未对齐 → 必须【拒绝转发】(防撞)
c=PositionCommand(); c.position.x=3.0; c.position.z=1.5; c.trajectory_flag=1
for _ in range(3): pcm.publish(c); rospy.sleep(0.2)   # 先让桥拿到 last_cmd
rospy.sleep(1.0)                                      # 等它彻底离开前段
sp.clear()                                            # 清掉前段残留
for _ in range(5): pcm.publish(c); rospy.sleep(0.1)
ck(len(sp)==0, "未对齐时拒绝转发（防撞保险）")
sys.exit(0 if all(F) else 1)
PY
cleanup; pkill -f "rosmaster --core -p 11351" 2>/dev/null

# ============================================================
#  实例2：测【后段行为】—— 例如朝向对齐 + 目标旋转
#        关键：让后段状态【立刻满足】(align_samples 设小)
# ============================================================
echo ""
echo "===== C) 实例2（align_samples=5，测对齐+旋转）====="
master 11352
setsid rosrun $PKG bridge_ctrl.py _auto_offboard:=false _seed_z:=0 _align_samples:=5 \
  >/tmp/sm_c2.log 2>&1 </dev/null &
sleep 5
pgrep -f bridge_ctrl.py >/dev/null && ok "控制桥在跑" || no "控制桥未起"

python3 - <<'PY' || FAIL=1
import math, sys, rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
rospy.init_node('sm2', anonymous=True, disable_signals=True)
F=[]
def ck(c,m): print("  %s %s" % ("PASS" if c else "FAIL", m)); F.append(c)
yq=lambda y:(0.0,0.0,math.sin(y/2),math.cos(y/2))     # 只绕 z 的四元数

sp=[]; rospy.Subscriber('/mavros/setpoint_position/local',PoseStamped,lambda m:sp.append(m))
po  = rospy.Publisher('/odom_world',Odometry,queue_size=10)
plp = rospy.Publisher('/mavros/local_position/pose',PoseStamped,queue_size=10)
pcm = rospy.Publisher('/planning/pos_cmd',PositionCommand,queue_size=10)
rospy.sleep(2)

PSI=math.radians(82.24)      # 模拟实测的两个坐标系朝向差
for _ in range(15):          # 喂够样本让桥估出 psi
    o=Odometry(); o.pose.pose.orientation.x,o.pose.pose.orientation.y,o.pose.pose.orientation.z,o.pose.pose.orientation.w = yq(0.0)
    po.publish(o)
    s=PoseStamped(); s.pose.orientation.x,s.pose.orientation.y,s.pose.orientation.z,s.pose.orientation.w = yq(PSI)
    plp.publish(s); rospy.sleep(0.1)
rospy.sleep(1)

c=PositionCommand(); c.position.x=3.0; c.position.y=0.0; c.position.z=1.5; c.trajectory_flag=1
sp.clear()
for _ in range(20): pcm.publish(c); rospy.sleep(0.1)
rospy.sleep(0.5)
if not sp:
    ck(False, "对齐后转发目标")
else:
    g=sp[-1]; ex=(3.0*math.cos(PSI), 3.0*math.sin(PSI))     # 解析解对拍
    ck(abs(g.pose.position.x-ex[0])<0.05 and abs(g.pose.position.y-ex[1])<0.05,
       "目标按 Rz(psi) 旋转 → (%.3f, %.3f)（期望 %.3f, %.3f）"
       % (g.pose.position.x,g.pose.position.y,ex[0],ex[1]))
    ey=math.degrees(2*math.atan2(g.pose.orientation.z,g.pose.orientation.w))
    ck(abs(ey-82.24)<1.0, "yaw 同步偏移 → %.2f deg（期望 82.24）" % ey)
sys.exit(0 if all(F) else 1)
PY
cleanup; pkill -f "rosmaster --core -p 11352" 2>/dev/null

echo ""
[ $FAIL -eq 0 ] && echo "SMOKE TEST: ALL PASS" || echo "SMOKE TEST: FAILED"
exit $FAIL
