# models/ — 各模型专属空间

## 设计原则：知识只有一份，适配各有一份

```
knowledge/            ← 唯一事实源，所有模型共用
models/
  core_prompt.md      ← 所有模型共用的角色与纪律
  claude/  gpt/  deepseek/
    adapter.md        ← 该模型专属的补充指令（只写"这个模型需要额外提醒的事"）
    notes.md          ← 观察到的该模型行为特点：擅长什么、常犯什么错
    inbox/            ← 该模型从 trajectory 中提炼的【候选】知识，待审核
    eval-results/     ← 该模型在 evals/ 上的历次成绩
```

**为什么不给每个模型各存一份知识库？** 三份副本一定会分叉，到时没人知道哪份是对的。模型之间真正不同的只有三件事：怎么说话最有效（adapter）、它的行为偏差（notes）、它自己发现的东西（inbox）。

## 候选知识的晋升流程

```
models/<m>/inbox/xxx.md
   │ 1. 人工审核：有证据吗？范围写清了吗？
   │ 2. 跑 evals：加入后成绩不下降（最好上升）
   ▼
knowledge/gap-ledger/ 或 knowledge/faults/
```

同一条知识被两个以上模型独立提出，是一个很强的信号，优先审核。

## 跨模型对比
`eval-results/` 用统一格式（见 `evals/README.md`），可以直接比较：同一份知识对哪个模型帮助最大、哪个模型更容易忽略检索到的知识。
