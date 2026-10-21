#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_explore_rviz.py — 从 FUEL 自带的 exp.rviz 派生一份「实机自主探索」RViz 配置

为什么要有这个脚本（而不是每次手搓）：
  1. exp.rviz 的显示树在 **Visualization Manager.Displays** 下面，不是顶层 `Displays`。
     取错层级 → find_class() 全返回 None → 新增项【静默丢失】，你以为加上了其实没有。
  2. exp.rviz 里【没有 rviz/Pose 模板】→ `/exploration_goal` 那项必须手写 dict。
  3. exp.rviz 自带的三个仿真专有组（Simulation / Estimation / Tracking）在实机上
     只会刷 "No message received"，删掉更干净。
  4. 实机上你最想看的两样（SUPER 航迹、FUEL 目标）exp.rviz 里都没有，要并进来。

用法（在飞机上跑，或在任何有这两个 rviz 文件的机器上跑）：
    python3 build_explore_rviz.py \
        --exp  <FUEL>/fuel_planner/plan_manage/config/exp.rviz \
        --top  <task_run_scripts>/rviz/mid360_top_down.rviz \
        --out  <task_run_scripts>/rviz/explore_fuel.rviz \
        [--frame world]
之后把 launch 的 rviz_config 指到 --out（RViz 只在启动时读配置，必须重启才生效）。

★ 校验：脚本最后会 re-parse 输出文件并打印顶层显示名，必须包含
  '★实机探索 (SUPER+FUEL) 附加显示'，否则说明有 bug。
"""
import argparse
import copy
import sys

import yaml

DROP_GROUPS = {"Simulation", "Estimation", "Tracking"}   # 仿真专有


def find_by_topic(displays, topic):
    """递归找带指定 Topic 的显示（rviz 里 key 是 'Topic'，偶尔小写）。"""
    for d in displays:
        t = d.get("Topic")
        if t is None:
            t = next((v for k, v in d.items() if k.lower() == "topic"), None)
        if t == topic:
            return d
        if d.get("Displays"):
            r = find_by_topic(d["Displays"], topic)
            if r:
                return r
    return None


def find_by_class(displays, klass):
    """递归找指定 Class 的显示（用作 deepcopy 模板）。"""
    for d in displays:
        if d.get("Class") == klass:
            return d
        if d.get("Displays"):
            r = find_by_class(d["Displays"], klass)
            if r:
                return r
    return None


def pose_item(name, topic, color):
    """手写一个 rviz/Pose（RViz 会自动补齐未给的默认键）。"""
    return {
        "Class": "rviz/Pose",
        "Name": name,
        "Enabled": True,
        "Topic": topic,
        "Unreliable": False,
        "Queue Size": 10,
        "Shape": "Arrow",
        "Color": color,
        "Alpha": 1,
        "Shaft Length": 0.8,
        "Head Length": 0.35,
        "Head Radius": 0.08,
        "Position Tolerance": 0.01,
        "Angle Tolerance": 0.01,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", required=True, help="FUEL 的 exp.rviz")
    ap.add_argument("--top", required=True, help="实机俯视图 rviz（用来摘 Path 模板）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", default="world")
    ap.add_argument("--keep-dropped", action="store_true",
                    help="保留 Simulation/Estimation/Tracking（默认删）")
    # 话题名可按实机改（默认就是 fouruav 的）
    ap.add_argument("--goal-topic", default="/exploration_goal")
    ap.add_argument("--released-topic", default="/waypoint_goal")
    ap.add_argument("--path-topic", default="/waypoint_mission")
    ap.add_argument("--supper-pc-topic", default="/fsm_node/visualization/replan_log_pc")
    a = ap.parse_args()

    exp = yaml.safe_load(open(a.exp))
    top = yaml.safe_load(open(a.top))

    # ★★ 走对层级：Visualization Manager.Displays
    vm = exp["Visualization Manager"]
    vm.setdefault("Global Options", {})["Fixed Frame"] = a.frame
    displays = vm.get("Displays", [])

    if a.keep_dropped:
        kept, removed = list(displays), []
    else:
        kept = [d for d in displays if d.get("Name") not in DROP_GROUPS]
        removed = [d.get("Name") for d in displays if d.get("Name") in DROP_GROUPS]

    # 从俯视图里把现成的 Path 显示整条 deepcopy 过来（省得手写 Path 的一堆键）
    added = []
    p = find_by_topic(top.get("Visualization Manager", {}).get("Displays", []), a.path_topic)
    if p:
        p = copy.deepcopy(p)
        p["Name"] = "SUPER航迹 (%s)" % a.path_topic
        p["Enabled"] = True
        added.append(p)

    # SUPER 规划点云：拿 kept 里现成的 rviz/PointCloud2 当模板
    pc_tpl = find_by_class(kept, "rviz/PointCloud2")
    if pc_tpl:
        x = copy.deepcopy(pc_tpl)
        x.update({"Name": "SUPER规划点云 (%s)" % a.supper_pc_topic,
                  "Enabled": True, "Topic": a.supper_pc_topic})
        added.append(x)

    # ★ 这两个是实机上最想看的：FUEL 想去哪 / SUPER 实际收到哪
    added.append(pose_item("★FUEL探索目标 (%s)" % a.goal_topic, a.goal_topic, "255; 60; 60"))
    added.append(pose_item("★仲裁器放行目标 (%s)" % a.released_topic, a.released_topic, "60; 255; 120"))

    group = {"Class": "rviz/Group",
             "Name": "★实机探索 (SUPER+FUEL) 附加显示",
             "Enabled": True, "Displays": added}
    vm["Displays"] = kept + [group]

    yaml.safe_dump(exp, open(a.out, "w"), default_flow_style=False,
                   allow_unicode=True, sort_keys=False)

    print("removed groups :", removed)
    print("kept groups    :", [d.get("Name") for d in kept])
    print("added          :", [d.get("Name") for d in added])
    print("written        :", a.out)

    # ★ 自校验：re-parse 并确认新增组真的在
    d2 = yaml.safe_load(open(a.out))
    names = [x.get("Name") for x in d2["Visualization Manager"]["Displays"]]
    print("RE-PARSE OK, top-level displays:", names)
    if group["Name"] not in names:
        print("!! FAIL: 新增组没写进去（八成是取错了 Displays 层级）")
        sys.exit(1)


if __name__ == "__main__":
    main()
