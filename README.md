# sim2real-copilot

An AI agent that helps close the sim-to-real gap in drone development, with log diagnosis, simulation testing, and a living knowledge base of sim-vs-real discrepancies.

无人机从仿真到实机全流程的 AI 开发伙伴。核心不是教模型"按步骤干活"，而是**积累模型学不到的垂域知识，并用 eval 证明它有用**。

## 仓库结构

```
sim2real-copilot/
├── knowledge/             专用知识库：唯一事实源，所有模型共用
│   ├── 00_handbook.md       总纲（先读这个）
│   ├── domains/             12 个领域知识文档 KB_*.md
│   ├── gap-ledger/          sim-to-real 差异台账（核心资产）
│   ├── faults/              故障经验库
│   ├── platform/            平台事实（机型、传感器、固件）
│   └── sources/             40 个原始技能 + 205 份配套文档（只读存档）
├── models/                各模型专属空间
│   ├── core_prompt.md       共用的角色、纪律与安全边界
│   ├── claude/              adapter / notes / inbox / eval-results
│   ├── gpt/
│   └── deepseek/
├── evals/                 评测：用例、运行脚本、候选用例
├── trajectories/          工作轨迹：知识与 eval 的原料
├── tools/                 脱敏、泄露检查、轨迹挖掘
├── CLAUDE.md              Claude Code 入口
└── AGENTS.md              GPT / Codex 等工具入口
```

## 核心闭环

```
  工作（任一模型） ──▶ trajectories/ ──▶ tools/mine_trajectories.py
                                              │
              ┌───────────────────────────────┴──────────────┐
              ▼                                              ▼
   models/<m>/inbox/ 候选知识                    evals/candidates/ 候选用例
              │ 人工审核                                     │ 补全判据
              ▼                                              ▼
        evals/run_eval.py  ◀────────── 成绩不下降才合并 ──────── evals/cases/
              │
              ▼
   knowledge/（gap-ledger / faults / domains）
```

详见 [docs/architecture.md](docs/architecture.md)。

## 快速开始

```bash
pip install -r requirements.txt
cp .env.example .env                                    # 填入 API key（不会被提交）

python evals/run_eval.py --space claude --dry-run       # 检查用例
python evals/run_eval.py --space deepseek --provider deepseek --model <MODEL_ID>
python evals/run_eval.py --space deepseek --provider deepseek --model <MODEL_ID> --no-knowledge   # 知识消融
```

- **Claude Code**：在仓库根目录启动，自动读取 `CLAUDE.md`
- **Codex 等**：自动读取 `AGENTS.md`
- **API 方式**：把 `models/core_prompt.md` 与 `models/<m>/adapter.md` 拼接为 system prompt

## 设计原则

1. **知识只有一份**：模型空间不复制知识，只放适配层、行为观察和候选知识。
2. **每条知识有来源、有范围、有状态**：见 [knowledge/README.md](knowledge/README.md)。
3. **用 eval 说话**：知识改动以 eval 成绩为准；`--no-knowledge` 消融量化知识价值。
4. **安全边界**：仿真自由操作；硬件在环需确认；实机只准备、不执行。
5. **汇报诚实**：已编写 / 离线通过 / 后端联通 / 实飞通过，四级分清。

## 关于脱敏

本仓库公开。所有口令、内网 IP、主机指纹、个人用户名都已替换为占位符（如 `<SSH_PASSWORD>`、`<NET_RC>.14`）。IP 只替换网段、保留主机号，"是否同网段"这类判据仍然成立。

提交前务必运行：

```bash
python tools/check_secrets.py .     # 结果必须为 0
```

## License

MIT
