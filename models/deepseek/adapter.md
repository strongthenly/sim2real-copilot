# DeepSeek 专属补充

DeepSeek 没有约定俗成的仓库入口文件。通过 API（OpenAI 兼容接口）或 Cline / Aider 等工具使用时，把 `models/core_prompt.md` 与本文件拼接为 system prompt；`evals/run_eval.py` 会自动完成拼接。

- 知识库以中文为主，优先用中文回答。
- 使用推理模式时，最终答案仍需包含出处和汇报等级。
- 产出候选知识时，写入 `models/deepseek/inbox/`。
