---
name: atomgit-pr
description: >
  自动化 AtomGit Pull Request 创建和语义化 PR 描述生成。
  使用 LLM 分析文件变更，无需硬编码规则即可生成准确描述。
  适用于：创建 Pull Request、自动生成 LLM 分析的 PR 描述、生成反映实际代码变更的语义化描述、或面向 AtomGit 仓库的操作。
license: MIT
---

# AtomGit Pull Request 创建工具

自动创建 Pull Request 并使用 LLM 分析生成语义化 PR 描述。

## 可用脚本

| 脚本 | 用途 |
|------|------|
| `generate_pr.py` | **推荐** - LLM 驱动的语义化 PR 描述生成 |
| `create_pr.py` | 本地仓库 PR 创建 |

## When to Use

Invoke this skill when:
- ✅ 创建 AtomGit Pull Request
- ✅ 自动生成 LLM 分析的 PR 描述
- ✅ 生成反映实际代码变更的语义化描述
- ✅ 面向 AtomGit 仓库的操作

Do NOT invoke for:
- ❌ 代码审查 (使用 `atomgit-code-review` skill)
- ❌ 架构审查 (使用 `atomgit-architecture-review` skill)
- ❌ 修复检视意见 (使用 `atomgit-code-review-repair` skill)

## 使用方法

### 生成语义化 PR 描述（推荐）

**IMPORTANT**: 所有命令必须从项目根目录运行，并使用环境设置：

```bash
# 切换到项目根目录
cd /home/xqw/Research/IB_Robot

# 生成 PR 描述
source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr 50

# 非交互模式（自动确认更新）
source .shrc_local && echo "y" | python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr 50

# 简单模式（不使用 LLM）
source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr 50 --simple
```

### 创建新 PR

```bash
source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/create_pr.py --branch feature-branch
```

## 设计理念

### LLM 驱动分析

使用 LLM 替代硬编码规则：

1. **理解实际代码变更** - 不再"凭空想象"不存在的功能
2. **适配任何代码库** - 无需修改即可适用于任何项目
3. **生成准确描述** - 真实反映代码的实际功能

### 动态变更类型

| 文件状态 | 类型标签 |
|----------|----------|
| `added` | 新增 |
| `renamed` | 移动 |
| `modified` | 修改 |
| `deleted` | 删除 |

## 输出示例

```markdown
### 主要变更

- **修改视频工具模块** (`src/lerobot/datasets/video_utils.py`)
  - 添加视频解码后端自动选择功能，支持基于图像的时间戳同步加载

- **新增视频处理** (`src/tools/preprocessor/video_to_images.py`)
  - 提供视频到图像的转换功能，支持 PyAV 和 Decort 解码，多进程处理与 GPU 加速

---

## 如何测试

**测试命令**

```bash
pytest tests/tools/preprocessor/test_preprocess_videos_pyav.py -v
pytest tests/tools/preprocessor/ -v
```

## 测试验证报告

### 语法检查输出

```bash
$ python -m py_compile tests/tools/preprocessor/test_*.py

=== 语法检查结果 ===
✓ tests/tools/preprocessor/test_preprocess_videos_pyav.py: 语法检查通过
✓ tests/tools/preprocessor/test_preprocess_videos_real.py: 语法检查通过
...
=== 语法检查完成 ===
所有文件语法检查通过 ✅
```

### 验证结论

| 验证项 | 状态 | 说明 |
|--------|------|------|
| Python 语法检查 | ✅ 通过 | 所有 7 个文件语法正确 |
| 测试框架结构 | ✅ 通过 | 4 个测试文件 |
| 测试用例覆盖 | ✅ 通过 | 覆盖主要功能场景 |
```

## 配置

在项目根目录创建 `config.json`：

```json
{
  "atomgit": {
    "token": "your_atomgit_token",
    "baseUrl": "https://api.atomgit.com",
    "owner": "openEuler",
    "repo": "IB_Robot"
  },
  "anthropic": {
    "apiKey": "sk-ant-..."
  }
}
```

### Claude API 配置（可选）

脚本会自动从 `~/.claude/settings.json` 读取 API 配置：

```json
{
  "env": {
    "ANTHROPIC_AUTH_TOKEN": "your_token_here",
    "ANTHROPIC_BASE_URL": "https://api.anthropic.com"
  }
}
```

支持自定义端点（如 `api.z.ai`）。

## 核心特性

### 智能特征提取

- 段落级分组，保持语义完整性
- 过滤问答格式和非功能性内容
- 仅保留包含功能性关键词的描述
- 每个文件最多 3 个核心特征

### 中文 PR 标题生成

- 使用 Claude API 生成简洁中文标题（10-50 字符）
- 格式：`[动作] [具体功能描述]`
- 示例：
  - ✅ `新增视频预处理工具，支持多进程解码`
  - ✅ `修复视频解码内存泄漏问题`
  - ❌ `更新代码结构`（过于泛泛）

### Markdown 格式测试命令

- 自动包装为代码块
- 清理冗余前缀文字
- 仅包含可执行的 bash 命令

## Troubleshooting

### Issue: ModuleNotFoundError when running script

**Root Cause**: Environment not set up properly

**Solution**:
```bash
cd /home/xqw/Research/IB_Robot
source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr 50
```

### Issue: Authentication failed (401 error)

**Root Cause**: Invalid or missing AtomGit token

**Solution**:
1. 验证 `config.json` 在项目根目录存在
2. 检查 token 有 `repo` 和 `pull_request` 权限
3. 确保 `baseUrl` 正确 (`https://api.atomgit.com`)

### Issue: LLM API Error

**Root Cause**: Missing or invalid Anthropic API key

**Solution**: Add `anthropic.apiKey` to config.json:
```json
{
  "anthropic": {
    "apiKey": "sk-ant-..."
  }
}
```

### Issue: PR not found (404 error)

**Root Cause**: Wrong PR number or repository configuration

**Solution**:
1. 验证 PR 号在 AtomGit 上存在
2. 检查 `config.json` 中 `owner` 和 `repo` 匹配实际仓库
3. 确保 token 有仓库访问权限

## Quick Reference

| Task | Command |
|------|---------|
| Generate PR desc | `source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr <number>` |
| Generate (simple) | `source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/generate_pr.py --pr <number> --simple` |
| Create new PR | `source .shrc_local && python3 .agents/skills/atomgit-pr/scripts/create_pr.py --branch <branch>` |
| Non-interactive | Add `echo "y" \|` before command |

## Related Skills

- **atomgit-code-review**: 代码审查，创建检视意见
- **atomgit-architecture-review**: IB_Robot 架构合规性审查
- **atomgit-code-review-repair**: 自动修复检视意见
- **ibrobot-env**: 处理环境设置

## References

- AtomGit API: https://docs.atomgit.com/docs/apis/
- Anthropic Claude API: https://docs.anthropic.com/
