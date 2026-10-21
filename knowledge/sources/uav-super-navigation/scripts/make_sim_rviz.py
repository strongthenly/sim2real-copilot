#!/usr/bin/env python3
"""从实机 RViz 配置派生「仿真版」，让一个窗口同时显示 点云 + rog_map + 规划轨迹。

为什么需要它：仿真脚本（super_explore_all.sh）直接调 fsm_node 二进制、绕过
real_flight.launch，于是实机那份 top_down.rviz 从不会自动加载；而直接拿实机那份用
也看不到点云 —— 它的点云项订阅的是实机 odin 话题 /odin1/cloud_slam。

用法（在 WSL 里跑）：
    python3 make_sim_rviz.py                       # 用下面的默认路径
    python3 make_sim_rviz.py <src.rviz> <dst.rviz>

默认：
    src = ~/super_sim_ws/src/task_run_scripts/rviz/top_down.rviz
    dst = ~/super_sim_ws/src/task_run_scripts/rviz/sim_top_down.rviz

⚠️ 必须用 YAML 读写，不要按行文本替换：
   RViz 配置里 `Decay Time:` / `Queue Size:` 这类键在多个显示项里重复出现，
   按「块边界」定位极易改错项（实测第一版就把 GlobalCloud 与 Occ 的 decay 互相覆盖，
   读回校验才发现）。safe_load/safe_dump 按显示项名字定位，不可能改错。

配套（缺了它点云照样不显示）：补 world→camera_init 静态 TF，
因为 /cloud_registered 的 frame 是 camera_init，而本配置 Fixed Frame 是 world。
    rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100
（仿真 sim_real_flight.yaml 的 fix_map_origin=[0,0,0]，两 frame 重合 → 零偏移。
  若改回 [0,0,1.5]，此处 z 必须同步改成 1.5）
"""
import os
import sys

import yaml

DEFAULT_SRC = os.path.expanduser(
    "~/super_sim_ws/src/task_run_scripts/rviz/top_down.rviz")
DEFAULT_DST = os.path.expanduser(
    "~/super_sim_ws/src/task_run_scripts/rviz/sim_top_down.rviz")

# 显示项名 -> (期望话题, 期望 Decay Time)
#   GlobalCloud  : 实机 /odin1/cloud_slam -> 仿真 /cloud_registered
#   decay 0      : 每帧独立不累积 -> 丢一帧就闪（稳定显示的是有累积的 surround）
CLOUD_WANT = {
    "GlobalCloud":   ("/cloud_registered", 1),
    "Occ":           ("/fsm_node/rog_map/occ", 2),
    "Unknown":       ("/fsm_node/rog_map/unk", 2),
    "InflatedCloud": ("/fsm_node/rog_map/inf_occ", 2),
}
FRAME_RATE = 15          # 点云本身才 10 Hz，30 帧纯浪费
REAL_ODIN_TOPIC = "/odin1/cloud_slam"


def walk(displays):
    """深度优先遍历显示项（含 rviz/Group 子项）。"""
    for it in displays or []:
        yield it
        if it.get("Displays") is not None:
            yield from walk(it["Displays"])


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SRC
    dst = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_DST
    if not os.path.isfile(src):
        sys.exit("源配置不存在: %s" % src)

    doc = yaml.safe_load(open(src))
    vm = doc["Visualization Manager"]
    log = []

    for it in walk(vm.get("Displays")):
        if it.get("Class") != "rviz/PointCloud2":
            continue
        name = it.get("Name")
        if name not in CLOUD_WANT:
            continue
        want_topic, want_decay = CLOUD_WANT[name]

        cur = it.get("Topic")
        if cur == REAL_ODIN_TOPIC or cur != want_topic:
            it["Topic"] = want_topic
            log.append("%-14s Topic %s -> %s" % (name, cur, want_topic))
        if it.get("Decay Time") != want_decay:
            log.append("%-14s Decay Time %s -> %s"
                       % (name, it.get("Decay Time"), want_decay))
            it["Decay Time"] = want_decay

    go = vm["Global Options"]
    if go.get("Frame Rate") != FRAME_RATE:
        log.append("Global Options Frame Rate %s -> %s"
                   % (go.get("Frame Rate"), FRAME_RATE))
        go["Frame Rate"] = FRAME_RATE

    yaml.safe_dump(doc, open(dst, "w"), default_flow_style=False,
                   allow_unicode=True, sort_keys=False, width=4096)

    print("源 : %s" % src)
    print("目标: %s" % dst)
    print("改动 (%d):" % len(log))
    for line in log:
        print("  - " + line)

    # --- 读回自校验 ---
    back = yaml.safe_load(open(dst))
    bvm = back["Visualization Manager"]
    got = {it["Name"]: (it.get("Topic"), it.get("Decay Time"))
           for it in walk(bvm.get("Displays"))
           if it.get("Class") == "rviz/PointCloud2" and it.get("Enabled")}
    bad = [k for k, v in CLOUD_WANT.items() if got.get(k) != v]
    frame = bvm["Global Options"].get("Fixed Frame")
    print("\n校验:")
    print("  Fixed Frame = %s  （应为 world）" % frame)
    for k, v in CLOUD_WANT.items():
        print("    %s %-14s %s" % ("OK " if got.get(k) == v else "BAD", k, got.get(k)))
    if bad or frame != "world":
        print("\n校验未通过: %s" % (bad or "Fixed Frame"))
        print("提醒: 点云 /cloud_registered 的 frame 是 camera_init，"
              "必须补 TF 才显示：")
        print("  rosrun tf static_transform_publisher 0 0 0 0 0 0 world camera_init 100")
        sys.exit(1)
    print("\n通过。别忘了在启动脚本里补 world->camera_init TF，并重启 RViz 才生效。")


if __name__ == "__main__":
    main()
