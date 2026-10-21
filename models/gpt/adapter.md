# GPT 专属补充

入口：仓库根目录的 `AGENTS.md`（Codex 等工具自动读取），它引用 `models/core_prompt.md` 和本文件。通过 API 使用时，把 core_prompt.md 与本文件拼接为 system prompt。

- 回答诊断类问题时，先列出现象分类，再给排查顺序，不要直接跳到修复。
- 产出候选知识时，写入 `models/gpt/inbox/`。
