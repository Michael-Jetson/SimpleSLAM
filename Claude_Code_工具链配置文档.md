# Claude Code 工具链配置文档

> 配置日期：2026-04-29（graphify 更新为 pip 安装方式）
> 环境：Ubuntu Linux / Node v24.14.1 / Python 3.12.3 / uv 0.11.8
> 订阅：Claude Max (Opus 4.6, 1M context)

---

## 一、MCP 服务器（9 个）

MCP (Model Context Protocol) 让 Claude 能直接调用外部工具和数据源。配置保存在 `~/.claude.json` 的 `mcpServers` 字段中。

### 通用管理命令

```bash
# 添加 MCP
claude mcp add <name> -s user -- <command> [args...]
# 带环境变量
claude mcp add <name> -s user -e KEY=VALUE -- <command> [args...]
# 移除
claude mcp remove <name> -s user
# 查看所有 MCP 状态
claude mcp list
```

---

### 1. arxiv — arXiv 论文搜索

- **功能**：搜索 arXiv 预印本论文、获取摘要/全文、按关键词/作者/分类检索、语义搜索、引用图谱
- **GitHub**：`blazickjp/arxiv-mcp-server`
- **安装方式**：

```bash
# 需要先安装 uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env

# 添加 MCP
claude mcp add arxiv -s user -- /home/gpf/.local/bin/uvx arxiv-mcp-server
```

- **环境变量**：无需
- **当前配置**：

```json
{
  "command": "/home/gpf/.local/bin/uvx",
  "args": ["arxiv-mcp-server"]
}
```

- **提供的工具**：`search_papers`, `read_paper`, `download_paper`, `get_abstract`, `semantic_search`, `citation_graph`, `watch_topic`, `check_alerts`, `list_papers`, `reindex`

---

### 2. semantic-scholar — Semantic Scholar 学术搜索

- **功能**：论文搜索、引用/被引分析、作者信息、论文推荐、全文获取、嵌入向量搜索。支持 33+ 个工具，含 PDF 转 Markdown
- **GitHub**：`hy20191108/semantic-scholar-mcp`（PyPI: `semantic-scholar-mcp`）
- **安装方式**：

```bash
claude mcp add semantic-scholar -s user -- /home/gpf/.local/bin/uvx semantic-scholar-mcp
```

- **环境变量**：可选设置 `SEMANTIC_SCHOLAR_API_KEY` 获得更高速率限制（免费申请：https://www.semanticscholar.org/product/api）
- **当前配置**：

```json
{
  "command": "/home/gpf/.local/bin/uvx",
  "args": ["semantic-scholar-mcp"]
}
```

- **注意**：首次启动较慢（55 个依赖包含 ONNX Runtime、PyMuPDF），后续缓存后正常
- **提供的工具**：`search_papers`, `get_paper`, `get_paper_citations`, `get_paper_references`, `get_paper_fulltext`, `get_paper_authors`, `get_recommendations_for_paper`, `search_authors`, `batch_get_papers`, `bulk_search_papers`, `search_snippets`, `get_paper_with_embeddings` 等 33 个

---

### 3. google-scholar — Google Scholar 搜索

- **功能**：Google Scholar 文献搜索、获取引用信息、BibTeX 导出、完整摘要获取。支持 CAPTCHA 自动弹出浏览器手动验证、Cookie 持久化
- **GitHub**：`arrogant-R/google_scholar_mcp`
- **安装方式**：

```bash
# 从 GitHub 安装（非 PyPI）
source ~/.local/bin/env
uv tool install git+https://github.com/arrogant-R/google_scholar_mcp.git

# 添加 MCP
claude mcp add google-scholar -s user -- /home/gpf/.local/bin/google_scholar_mcp
```

- **环境变量**：无需
- **当前配置**：

```json
{
  "command": "/home/gpf/.local/bin/google_scholar_mcp",
  "args": []
}
```

- **提供的工具**：`search_google_scholar`

---

### 4. github — GitHub 仓库管理

- **功能**：管理 GitHub 仓库、创建/查看 Issues 和 PR、搜索代码、管理分支、文件操作
- **GitHub**：`modelcontextprotocol/servers`（官方）
- **安装方式**：

```bash
claude mcp add github -s user -- npx -y @modelcontextprotocol/server-github

# 如需写操作，需添加 Token：
claude mcp remove github -s user
claude mcp add github -s user -e GITHUB_PERSONAL_ACCESS_TOKEN=你的Token -- npx -y @modelcontextprotocol/server-github
```

- **环境变量**：`GITHUB_PERSONAL_ACCESS_TOKEN`（可选，无 Token 时只读）
  - 获取：GitHub Settings → Developer settings → Personal access tokens
- **当前配置**：

```json
{
  "command": "npx",
  "args": ["-y", "@modelcontextprotocol/server-github"]
}
```

- **提供的工具**：`search_repositories`, `search_code`, `get_file_contents`, `create_issue`, `create_pull_request`, `list_commits`, `push_files` 等 25+ 个

---

### 5. docker — Docker 容器管理

- **功能**：Docker 容器/镜像/卷/网络管理，支持 Docker Compose 部署、日志查看
- **GitHub**：`ckreiling/mcp-server-docker`（PyPI: `mcp-server-docker`）
- **安装方式**：

```bash
claude mcp add docker -s user -- sg docker -c "/home/gpf/.local/bin/uvx mcp-server-docker"
```

- **环境变量**：无需，通过 Docker socket 通信
- **当前配置**：

```json
{
  "command": "sg",
  "args": ["docker", "-c", "/home/gpf/.local/bin/uvx mcp-server-docker"]
}
```

- **注意**：使用 `sg docker -c` 包裹命令以解决 Docker socket 权限问题。如果当前用户不在 docker 组，需先运行 `sudo usermod -aG docker $USER` 并重新登录
- **提供的工具**：`list_containers`, `create_container`, `run_container`, `start_container`, `stop_container`, `remove_container`, `fetch_container_logs`, `list_images`, `pull_image`, `build_image`, `list_volumes`, `list_networks` 等 19 个

---

### 6. jupyter — Jupyter Notebook 操作

- **功能**：连接 JupyterLab、执行代码、管理 Notebook 单元格、多模态输出（图片/图表）
- **GitHub**：`datalayer/jupyter-mcp-server`（PyPI: `jupyter-mcp-server`）
- **安装方式**：

```bash
claude mcp add jupyter -s user -- /home/gpf/.local/bin/uvx jupyter-mcp-server
```

- **环境变量**：使用时需设置：
  - `JUPYTER_URL`：Jupyter 服务器地址（如 `http://localhost:8888`）
  - `JUPYTER_TOKEN`：Jupyter 认证 Token
- **前置要求**：需先启动 JupyterLab：

```bash
pip install jupyterlab
jupyter lab --port 8888 --IdentityProvider.token MY_TOKEN
```

- **当前配置**：

```json
{
  "command": "/home/gpf/.local/bin/uvx",
  "args": ["jupyter-mcp-server"]
}
```

- **提供的工具**：`connect_to_jupyter`, `use_notebook`, `execute_cell`, `execute_code`, `read_cell`, `edit_cell_source`, `insert_cell`, `delete_cell`, `list_notebooks`, `list_kernels` 等 18 个

---

### 7. memory — 持久化知识图谱

- **功能**：跨会话持久化存储实体和关系，构建知识图谱
- **GitHub**：`modelcontextprotocol/servers`（官方）
- **安装方式**：

```bash
claude mcp add memory -s user -- npx -y @modelcontextprotocol/server-memory
```

- **环境变量**：无需
- **当前配置**：

```json
{
  "command": "npx",
  "args": ["-y", "@modelcontextprotocol/server-memory"]
}
```

- **提供的工具**：`create_entities`, `create_relations`, `add_observations`, `delete_entities`, `delete_relations`, `delete_observations`, `read_graph`, `search_nodes`, `open_nodes`

---

### 8. brave-search — Brave 网络搜索

- **功能**：通过 Brave Search API 搜索互联网，返回网页结果
- **GitHub**：`modelcontextprotocol/servers`（官方）
- **安装方式**：

```bash
claude mcp add brave-search -s user -e BRAVE_API_KEY=你的Key -- npx -y @modelcontextprotocol/server-brave-search
```

- **环境变量**：`BRAVE_API_KEY`（必须）
  - 获取：https://brave.com/search/api/ 注册免费 Key
  - 免费额度：Web Search API 每月 2,000 次
- **当前配置**：

```json
{
  "command": "npx",
  "args": ["-y", "@modelcontextprotocol/server-brave-search"],
  "env": {
    "BRAVE_API_KEY": "BSAVcrIWiOrMfh98EsdiPngHSqezxcw"
  }
}
```

---

### 9. codex — OpenAI Codex CLI 桥接

- **功能**：将 OpenAI Codex CLI 的能力桥接到 Claude Code，提供 AI 代码分析、代码审查、网络搜索
- **GitHub**：`tuannvm/codex-mcp-server`
- **前置要求**：需先安装 Codex CLI 并登录

```bash
npm i -g @openai/codex
codex login --api-key "你的OpenAI-API-Key"
```

- **安装方式**：

```bash
claude mcp add codex -s user -- npx -y codex-mcp-server
```

- **环境变量**：Codex CLI 自行管理认证
- **当前配置**：

```json
{
  "command": "npx",
  "args": ["-y", "codex-mcp-server"]
}
```

- **提供的工具**：`codex`（AI 助手）, `review`（代码审查）, `websearch`（网络搜索）, `listSessions`, `ping`, `help`

---

## 二、插件（13 个）

插件通过 Claude Code 的插件市场系统管理。配置保存在 `~/.claude/settings.json` 的 `enabledPlugins` 和 `extraKnownMarketplaces` 字段中。

### 通用管理命令

```bash
# 添加第三方市场
claude plugin marketplace add <owner>/<repo>
# 从市场安装
claude plugin install <plugin-name>@<marketplace-name>
# 从官方市场安装
claude plugin install <plugin-name>@claude-plugins-official
# 查看已安装
claude plugin list
# 卸载
claude plugin uninstall <plugin-name>@<marketplace-name>
# 禁用/启用
claude plugin disable <plugin-name>@<marketplace-name>
claude plugin enable <plugin-name>@<marketplace-name>
# 重载（安装后立即生效）
/reload-plugins
```

---

### 1. everything-claude-code (169K stars)

- **功能**：综合性 Agent 优化系统。48 个专业 Agent + 183 个 Skills + 79 个命令 + 自带 6 个 MCP（GitHub、Context7、Exa、Memory、Playwright、Sequential Thinking）
- **GitHub**：`affaan-m/everything-claude-code`
- **安装方式**：

```bash
claude plugin marketplace add affaan-m/everything-claude-code
claude plugin install everything-claude-code@everything-claude-code
```

- **版本**：1.10.0
- **自带 MCP**：github, context7, exa, memory, playwright, sequential-thinking

---

### 2. claude-mem (69K stars)

- **功能**：自动捕获 Claude 编码过程中的所有决策、推理和实现细节，形成可搜索的会话记忆
- **GitHub**：`thedotmack/claude-mem`
- **安装方式**：

```bash
claude plugin marketplace add thedotmack/claude-mem
claude plugin install claude-mem@thedotmack
```

- **版本**：12.4.8
- **注意**：自带一个 mcp-search MCP 服务器，需要 bun 运行。安装 bun：`curl -fsSL https://bun.sh/install | bash`

---

### 3. caveman (49K stars)

- **功能**：压缩 Claude 输出 token 消耗约 65%，让回复更精简高效。"why use many token when few token do trick"
- **GitHub**：`JuliusBrussee/caveman`
- **安装方式**：

```bash
claude plugin marketplace add JuliusBrussee/caveman
claude plugin install caveman@caveman
```

- **版本**：84cc3c14fa1e

---

### 4. superpowers (476K 安装，官方市场)

- **功能**：头脑风暴 + 子 Agent 驱动开发 + 代码审查。增强 Claude Code 的多 Agent 协作能力
- **来源**：Anthropic 官方插件市场
- **安装方式**：

```bash
claude plugin install superpowers@claude-plugins-official
```

- **版本**：5.0.7

---

### 5. context7 (269K 安装，官方市场)

- **功能**：实时拉取最新版本的库/框架文档，大幅减少 AI 幻觉。查询 PyTorch、NumPy 等库的最新 API
- **来源**：Anthropic 官方插件市场
- **安装方式**：

```bash
claude plugin install context7@claude-plugins-official
```

- **自带 MCP**：`@upstash/context7-mcp`

---

### 6. code-review (255K 安装，官方市场)

- **功能**：多 Agent 团队自动代码审查，带置信度评分
- **来源**：Anthropic 官方插件市场
- **安装方式**：

```bash
claude plugin install code-review@claude-plugins-official
```

---

### 7. planning-with-files (20K stars)

- **功能**：Manus 式持久化 Markdown 规划，将项目计划写入文件系统，支持复杂多步骤项目管理
- **GitHub**：`OthmanAdi/planning-with-files`
- **安装方式**：

```bash
claude plugin marketplace add OthmanAdi/planning-with-files
claude plugin install planning-with-files@planning-with-files
```

- **版本**：2.35.0

---

### 8. compound-engineering (16K stars)

- **功能**：复合工程实践插件，多 Agent 协作编码，覆盖完整工程生命周期
- **GitHub**：`EveryInc/compound-engineering-plugin`
- **安装方式**：

```bash
claude plugin marketplace add EveryInc/compound-engineering-plugin
claude plugin install compound-engineering@compound-engineering-plugin
```

- **版本**：3.2.0

---

### 9. autoresearch (4.1K stars)

- **功能**：自主目标驱动迭代研究，受 Karpathy 启发。Claude 自主探索、验证假设、迭代改进
- **GitHub**：`uditgoenka/autoresearch`
- **安装方式**：

```bash
claude plugin marketplace add uditgoenka/autoresearch
claude plugin install autoresearch@autoresearch
```

- **版本**：2.0.0

---

### 10. pyright-lsp (72K 安装，官方市场)

- **功能**：Python 语言服务器（Pyright），提供实时类型检查和代码智能分析
- **来源**：Anthropic 官方插件市场
- **安装方式**：

```bash
claude plugin install pyright-lsp@claude-plugins-official
```

- **版本**：1.0.0

---

### 11. claude-scientific-writer (1.6K stars)

- **功能**：生成可发表的科学论文、报告、海报、基金申请书、文献综述
- **GitHub**：`K-Dense-AI/claude-scientific-writer`
- **安装方式**：

```bash
claude plugin marketplace add K-Dense-AI/claude-scientific-writer
claude plugin install claude-scientific-writer@claude-scientific-writer
```

- **版本**：5bf6b597e2af

---

### 12. math-olympiad (官方市场)

- **功能**：竞赛数学和算法问题求解（IMO/Putnam/USAMO），带对抗验证机制
- **来源**：Anthropic 官方插件市场
- **安装方式**：

```bash
claude plugin install math-olympiad@claude-plugins-official
```

---

### 13. claude-hud

- **功能**：状态栏 HUD 显示，展示上下文使用量、活动工具、会话信息
- **GitHub**：`jarrodwatts/claude-hud`
- **安装方式**：

```bash
claude plugin marketplace add jarrodwatts/claude-hud
claude plugin install claude-hud@claude-hud
```

- **版本**：0.1.0
- **额外配置**：需要在 `~/.claude/settings.json` 中配置 `statusLine`（通过 `/claude-hud:setup` 自动完成）

---

## 三、手动安装的 Skills（7 个模块，140+ 技能）

Skills 是 Markdown 文件形式的提示词模板，安装在 `~/.claude/skills/` 目录下。Claude Code 会自动加载这些技能。

### 通用安装方法

```bash
# 方法 1：从 GitHub 克隆后复制
git clone --depth 1 https://github.com/<owner>/<repo>.git /tmp/<repo>
cp -r /tmp/<repo>/<skill-dir> ~/.claude/skills/
rm -rf /tmp/<repo>

# 方法 2：直接下载 SKILL.md
mkdir -p ~/.claude/skills/<skill-name>
curl -sL "https://raw.githubusercontent.com/<owner>/<repo>/<branch>/<path>/SKILL.md" \
  > ~/.claude/skills/<skill-name>/SKILL.md
```

---

### 1-4. academic-research-skills (3.7K stars)

- **功能**：完整学术论文工作流水线
- **GitHub**：`Imbad0202/academic-research-skills`
- **包含 4 个子模块**：

| 模块 | 路径 | 功能 |
|---|---|---|
| **academic-paper** | `~/.claude/skills/academic-paper/` | 12-Agent 论文撰写，支持 APA/IEEE/Chicago 引用格式、LaTeX/DOCX/PDF 输出 |
| **academic-paper-reviewer** | `~/.claude/skills/academic-paper-reviewer/` | 7-Agent 同行评审模拟，多维度论文质量评估 |
| **academic-pipeline** | `~/.claude/skills/academic-pipeline/` | 完整学术管线：research → write → review → revise → finalize |
| **deep-research** | `~/.claude/skills/deep-research/` | 13-Agent 深度研究，系统性文献调研 |

- **共享资源**：`~/.claude/skills/shared/`（引用格式模板、交叉验证工具等 16 个文件）
- **安装方式**：

```bash
git clone --depth 1 https://github.com/Imbad0202/academic-research-skills.git /tmp/ars
cp -r /tmp/ars/academic-paper ~/.claude/skills/
cp -r /tmp/ars/academic-paper-reviewer ~/.claude/skills/
cp -r /tmp/ars/academic-pipeline ~/.claude/skills/
cp -r /tmp/ars/deep-research ~/.claude/skills/
cp -r /tmp/ars/shared ~/.claude/skills/
rm -rf /tmp/ars
```

- **可选依赖**：
  - Pandoc（DOCX 输出）：`sudo apt install pandoc`
  - tectonic（PDF 输出）：`curl --proto '=https' --tlsv1.2 -fsSL https://drop-sh.fullyjustified.net | sh`

---

### 5. scientific-skills (19.6K stars)

- **功能**：134 个科研领域技能，覆盖生物信息学、化学、物理、工程仿真、机器学习等
- **GitHub**：`K-Dense-AI/claude-scientific-skills`
- **安装路径**：`~/.claude/skills/scientific-skills/`
- **与机器人/AI 最相关的技能**：

| 技能 | 功能 |
|---|---|
| `pytorch-lightning` | PyTorch Lightning 训练框架 |
| `torch-geometric` | 图神经网络 |
| `torchdrug` | 药物发现的 PyTorch 扩展 |
| `optimize-for-gpu` | GPU 优化 |
| `deepchem` | 深度化学/分子计算 |
| `latex-posters` | LaTeX 海报制作 |
| `literature-review` | 文献综述 |
| `citation-management` | 引文管理 |
| `paper-lookup` / `paperzilla` / `bgpt-paper-search` | 论文检索 |
| `peer-review` | 同行评审 |
| `pylabrobot` | 实验室机器人自动化 |

- **安装方式**：

```bash
git clone --depth 1 https://github.com/K-Dense-AI/claude-scientific-skills.git /tmp/css
cp -r /tmp/css/scientific-skills ~/.claude/skills/scientific-skills
rm -rf /tmp/css
```

---

### 6. graphify (38K stars)

- **功能**：将代码、文档、论文、图片、视频转为可查询的知识图谱。25+ 语言 tree-sitter AST 解析 + Claude Vision 多模态提取。71.5× fewer tokens per query vs 读原始文件
- **GitHub**：`safishamsi/graphify`（⭐ 38,340）
- **安装路径**：`~/.claude/skills/graphify/`
- **安装方式**（推荐 pip）：

```bash
# 安装 Python 工具 + Claude Code Skill（一条命令）
pip install graphifyy && graphify install
```

> PyPI 包名暂为 `graphifyy`（`graphify` 名称回收中），CLI 命令仍为 `graphify`。

- **安装结果**：
  - Skill 文件：`~/.claude/skills/graphify/SKILL.md`
  - 自动更新 `~/.claude/CLAUDE.md`（添加 graphify 触发规则）

- **使用方式**：

```bash
# 在 Claude Code 中输入：
/graphify .                        # 当前目录构建知识图谱
/graphify ./include                # 指定目录
/graphify ./raw --mode deep        # 更激进的推断边提取
/graphify ./raw --update           # 仅处理变化文件，增量合并
/graphify ./raw --wiki             # 生成 agent 可爬取的 wiki
/graphify ./raw --watch            # 文件变化时自动同步图谱

# 查询：
/graphify query "what connects ICP to the voxel map?"
/graphify path "OdometryBase" "LoopClosureService"
/graphify explain "RegistrationTarget"

# 添加外部资源：
/graphify add https://arxiv.org/abs/2410.08935   # 抓取论文并更新图谱

# Git Hook（每次 commit 自动重建）：
graphify hook install
```

- **输出目录**：`graphify-out/`

| 文件 | 说明 |
|------|------|
| `graph.html` | 交互式图谱（点击节点、搜索、按社区过滤） |
| `obsidian/` | 可直接作为 Obsidian Vault 打开 |
| `wiki/` | Wikipedia 风格文章，供 Agent 导航 |
| `GRAPH_REPORT.md` | God Nodes、意外连接、建议问题 |
| `graph.json` | 持久化图谱数据（支持离线查询） |
| `cache/` | SHA256 缓存（增量处理，只处理变化文件） |

- **支持的文件类型**：

| 类型 | 扩展名 | 提取方式 |
|------|--------|---------|
| 代码 | `.py .ts .js .go .rs .java .c .cpp .rb .cs .kt` | tree-sitter AST + 调用图 |
| 文档 | `.md .txt .rst` | Claude 概念+关系提取 |
| 论文 | `.pdf` | 引用挖掘 + 概念提取 |
| 图片 | `.png .jpg .webp .gif` | Claude Vision（截图、白板、任意语言） |

---

## 四、插件自带的 MCP 服务器（8 个）

以下 MCP 由已安装的插件自动提供，无需单独配置：

| 来源插件 | MCP 名称 | 功能 |
|---|---|---|
| everything-claude-code | `github` | GitHub 仓库管理 (v2025.4.8) |
| everything-claude-code | `context7` | 库文档查询 (v2.1.4) |
| everything-claude-code | `exa` | Exa AI 搜索 |
| everything-claude-code | `memory` | 知识图谱 (v2026.1.26) |
| everything-claude-code | `playwright` | 浏览器自动化 (v0.0.69) |
| everything-claude-code | `sequential-thinking` | 结构化思考 |
| context7 | `context7` | 库文档查询 |
| claude-mem | `mcp-search` | 会话记忆搜索（需 bun） |

---

## 五、预装的内置 MCP（需认证）

| MCP | 状态 | 说明 |
|---|---|---|
| Hugging Face | 需认证 | Claude.ai 自带，通过 `/mcp` 认证 |
| Google Drive | 需认证 | Claude.ai 自带，通过 `/mcp` 认证 |

---

## 六、环境依赖总结

### 已安装

| 工具 | 版本 | 用途 |
|---|---|---|
| Node.js | v24.14.1 | npx 类 MCP 服务器 |
| Python | 3.12.3 | uvx 类 MCP 服务器 |
| uv / uvx | 0.11.8 | Python 包管理 |
| Codex CLI | 0.125.0 | Codex MCP |
| Docker | 29.4.1 | Docker MCP |

### 建议安装

| 工具 | 安装命令 | 用途 |
|---|---|---|
| bun | `curl -fsSL https://bun.sh/install \| bash` | claude-mem MCP 搜索功能 |
| pandoc | `sudo apt install pandoc` | 论文 DOCX 输出 |
| tectonic | 见上文 | 论文 PDF/LaTeX 输出 |
| gh (GitHub CLI) | `sudo apt install gh` | GitHub 操作增强 |

---

## 七、配置文件位置

| 文件 | 内容 |
|---|---|
| `~/.claude.json` | MCP 服务器配置、项目数据、用户设置 |
| `~/.claude/settings.json` | 主题、权限、插件、statusLine、市场 |
| `~/.claude/skills/` | 手动安装的 Skills |
| `~/.claude/plugins/` | 插件缓存和注册表 |
| `~/.claude/commands/` | 自定义 Slash Commands（可自建） |

---

## 八、快速恢复指南

如需在新机器上恢复此配置：

```bash
# 1. 安装基础工具
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env

# 2. 安装 Google Scholar MCP（需从 git 安装）
uv tool install git+https://github.com/arrogant-R/google_scholar_mcp.git

# 3. 安装 graphify（pip 方式，自动注册 Skill）
pip install graphifyy && graphify install

# 4. 添加所有 MCP
claude mcp add arxiv -s user -- ~/.local/bin/uvx arxiv-mcp-server
claude mcp add semantic-scholar -s user -- ~/.local/bin/uvx semantic-scholar-mcp
claude mcp add google-scholar -s user -- ~/.local/bin/google_scholar_mcp
claude mcp add github -s user -- npx -y @modelcontextprotocol/server-github
claude mcp add docker -s user -- sg docker -c "~/.local/bin/uvx mcp-server-docker"
claude mcp add jupyter -s user -- ~/.local/bin/uvx jupyter-mcp-server
claude mcp add memory -s user -- npx -y @modelcontextprotocol/server-memory
claude mcp add brave-search -s user -e BRAVE_API_KEY=你的Key -- npx -y @modelcontextprotocol/server-brave-search
claude mcp add codex -s user -- npx -y codex-mcp-server

# 5. 安装插件市场
claude plugin marketplace add affaan-m/everything-claude-code
claude plugin marketplace add K-Dense-AI/claude-scientific-writer
claude plugin marketplace add JuliusBrussee/caveman
claude plugin marketplace add OthmanAdi/planning-with-files
claude plugin marketplace add EveryInc/compound-engineering-plugin
claude plugin marketplace add uditgoenka/autoresearch
claude plugin marketplace add thedotmack/claude-mem
claude plugin marketplace add jarrodwatts/claude-hud

# 6. 安装所有插件
claude plugin install everything-claude-code@everything-claude-code
claude plugin install claude-scientific-writer@claude-scientific-writer
claude plugin install caveman@caveman
claude plugin install planning-with-files@planning-with-files
claude plugin install compound-engineering@compound-engineering-plugin
claude plugin install autoresearch@autoresearch
claude plugin install claude-mem@thedotmack
claude plugin install claude-hud@claude-hud
claude plugin install superpowers@claude-plugins-official
claude plugin install code-review@claude-plugins-official
claude plugin install context7@claude-plugins-official
claude plugin install pyright-lsp@claude-plugins-official
claude plugin install math-olympiad@claude-plugins-official

# 7. 安装手动 Skills
mkdir -p ~/.claude/skills

git clone --depth 1 https://github.com/Imbad0202/academic-research-skills.git /tmp/ars
cp -r /tmp/ars/{academic-paper,academic-paper-reviewer,academic-pipeline,deep-research,shared} ~/.claude/skills/
rm -rf /tmp/ars

git clone --depth 1 https://github.com/K-Dense-AI/claude-scientific-skills.git /tmp/css
cp -r /tmp/css/scientific-skills ~/.claude/skills/
rm -rf /tmp/css

pip install graphifyy && graphify install
```
