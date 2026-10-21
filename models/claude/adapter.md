# Claude 专属补充

入口：仓库根目录的 `CLAUDE.md`（Claude Code 自动读取），它引用 `models/core_prompt.md` 和本文件。

- 知识文件很大时（`KB_control-stack.md` 约 44 万字节），先用搜索定位章节再读取片段，不要整份读入。
- 修改机载代码或配置时，遵循总纲 4.2：先备份、贴 diff、留回退方法。
- 产出候选知识时，写入 `models/claude/inbox/`。
