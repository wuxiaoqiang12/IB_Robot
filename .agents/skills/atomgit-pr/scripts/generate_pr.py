#!/usr/bin/env python3
"""
LLM 驱动的语义化 PR 描述生成工具

使用 LLM API 分析文件修改内容，生成准确的 PR 描述

用法：
    python3 generate_pr.py --pr <pr_number> [--simple] [--dry-run]
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Optional
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent.parent / "lib"))

from atomgit_api import AtomGitAPI, AtomGitConfig


def load_config(config_path: str = "config.json") -> dict:
    """加载配置文件"""
    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    if not config.get("atomgit") or not config["atomgit"].get("token"):
        raise Exception("配置文件中缺少 atomgit.token")

    return config


def get_llm_api_key() -> Optional[str]:
    """获取 LLM API 密钥"""
    api_key = os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("OPENAI_API_KEY")

    if not api_key:
        try:
            settings_path = os.path.expanduser("~/.claude/settings.json")
            if os.path.exists(settings_path):
                with open(settings_path, "r") as f:
                    settings = json.load(f)
                    api_key = settings.get("env", {}).get("ANTHROPIC_AUTH_TOKEN")
        except:
            pass

    return api_key


def call_llm(prompt: str, api_key: str) -> str:
    """调用 LLM API"""
    import anthropic

    base_url = os.environ.get("ANTHROPIC_BASE_URL")

    client_options = {"api_key": api_key}
    if base_url:
        client_options["base_url"] = base_url

    client = anthropic.Anthropic(**client_options)

    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=2000,
        system="你是一个代码分析专家，擅长总结代码修改的功能点。请用简洁、准确的中文回答。",
        messages=[{"role": "user", "content": prompt}],
    )

    return response.content[0].text.strip()


def analyze_file_with_llm(
    filename: str, content: str, api_key: str, status: str
) -> str:
    """使用 LLM 分析文件修改内容"""
    if not content or not api_key:
        return ""

    max_content_length = 3000
    truncated_content = (
        content[:max_content_length] + "\n... (内容已截断)"
        if len(content) > max_content_length
        else content
    )

    status_hint = {
        "renamed": "注意：这个文件是重命名的，主要变化是路径或名称改变。",
        "modified": "注意：这个文件是修改的，请总结主要修改内容。",
        "added": "注意：这个文件是新添加的。",
    }.get(status, "")

    prompt = f"""请分析以下 Python 文件的修改内容，总结其主要功能点：

文件名: {filename}
文件状态: {status or "unknown"}
{status_hint}

文件内容:
```python
{truncated_content}
```

请用简洁的中文回答：
1. 这个文件的主要功能是什么？（不超过50字）
2. 如果是修改/重命名，主要变化是什么？
3. 如果添加了新参数或功能，请说明。

格式：
- 如果是新文件：直接返回功能描述
- 如果是修改/重命名：说明主要变化
- 不要有其他说明文字。"""

    try:
        return call_llm(prompt, api_key)
    except Exception as e:
        print(f"  [LLM 分析失败: {e}]")
        return ""


def get_change_type(status: str) -> str:
    """获取文件变更类型"""
    return {
        "added": "新增",
        "renamed": "移动",
        "modified": "修改",
        "deleted": "删除",
    }.get(status, "修改")


def generate_semantic_changes(
    files: List[dict], api: AtomGitAPI, llm_api_key: str, branch: str
) -> List[dict]:
    """生成语义化变更描述"""
    changes = []
    source_changes = []
    test_changes = []

    changed_files = [f for f in files if f.get("status") != "deleted"]

    non_code_files = [
        ".gitignore",
        ".gitattributes",
        ".dockerignore",
        "license",
        "license.md",
        "readme.md",
    ]
    filtered_files = [
        f
        for f in changed_files
        if f.get("filename", "").split("/")[-1].lower() not in non_code_files
    ]

    if not filtered_files:
        return changes

    grouped = {}
    for f in filtered_files:
        dir_path = f.get("filename", "")
        if "/" in dir_path:
            dir_path = dir_path[: dir_path.rfind("/")]
        else:
            dir_path = ""

        if dir_path not in grouped:
            grouped[dir_path] = []
        grouped[dir_path].append(f)

    for dir_path in sorted(grouped.keys()):
        files_in_dir = grouped[dir_path]

        is_test_dir = (
            dir_path.startswith("tests/")
            or "/tests/" in dir_path
            or any(f.get("filename", "").startswith("tests/") for f in files_in_dir)
        )

        if is_test_dir:
            all_features = []

            for file in files_in_dir:
                filename = file.get("filename", "")
                if filename.endswith(".py") and "__init__" not in filename:
                    try:
                        content = api.get_file_content(filename, branch)
                        if llm_api_key and content:
                            llm_result = analyze_file_with_llm(
                                filename, content, llm_api_key, file.get("status")
                            )
                            if llm_result:
                                all_features.append(llm_result)
                    except:
                        pass

            test_changes.append(
                {
                    "type": "新增",
                    "name": "完整的测试套件",
                    "path": dir_path,
                    "main_file": f"{dir_path}/",
                    "features": list(set(all_features)),
                    "is_test": True,
                }
            )
        else:
            main_py_files = [
                f
                for f in files_in_dir
                if f.get("filename", "").endswith(".py")
                and "__init__" not in f.get("filename", "")
            ]

            if not main_py_files:
                continue

            main_file = main_py_files[0]
            filename = main_file.get("filename", "")

            try:
                content = api.get_file_content(filename, branch)
            except:
                content = ""

            change_type = get_change_type(main_file.get("status"))
            features = []
            feature_name = ""

            if llm_api_key and content:
                llm_result = analyze_file_with_llm(
                    filename, content, llm_api_key, main_file.get("status")
                )
                if llm_result:
                    features = [llm_result]
                    parts = filename.split("/")
                    name = parts[-1].replace(".py", "").replace("_", " ")
                    feature_name = name.capitalize()

            if not feature_name:
                parts = filename.split("/")
                name = parts[-1].replace(".py", "").replace("_", " ")
                feature_name = name.capitalize()

            source_changes.append(
                {
                    "type": change_type,
                    "name": feature_name,
                    "path": dir_path,
                    "main_file": filename,
                    "features": features,
                    "is_test": False,
                }
            )

    changes.extend(source_changes)
    changes.extend(test_changes)

    return changes


def generate_test_section(files: List[dict]) -> str:
    """生成测试说明"""
    test_files = [
        f
        for f in files
        if f.get("filename", "").startswith("tests/")
        and f.get("filename", "").endswith(".py")
        and "conftest" not in f.get("filename", "")
    ]

    if not test_files:
        return "- 验证代码编译/构建成功\n- 手动测试相关功能"

    test_dir = test_files[0].get("filename", "")[
        : test_files[0].get("filename", "").rfind("/")
    ]

    lines = ["**测试命令**\n", "```bash"]
    lines.append(f"pytest {test_dir}/ -v")
    lines.append("```")

    return "\n".join(lines)


def generate_pr_description(changes: List[dict], files: List[dict]) -> str:
    """生成完整的 PR 描述"""
    description = "### 主要变更\n\n"

    for change in changes:
        is_test_suite = change.get("main_file", "").endswith("/")

        if is_test_suite:
            description += (
                f"- **{change['type']}{change['name']}** (`{change['path']}/`)\n"
            )
        else:
            description += (
                f"- **{change['type']}{change['name']}** (`{change['main_file']}`)\n"
            )

        for feature in change.get("features", []):
            description += f"  - {feature}\n"

        description += "\n"

    description += "---\n\n## 如何测试\n\n"
    description += generate_test_section(files)

    description += "\n\n---\n\n"
    description += "**注意**: 社区中的任何人都可以在测试通过后审查 PR。欢迎标记对你这个 PR 感兴趣的成员/贡献者。尽量避免标记超过 3 个人。\n"

    return description


def main():
    parser = argparse.ArgumentParser(description="LLM 驱动的语义化 PR 描述生成")
    parser.add_argument("--pr", type=int, required=True, help="PR 编号")
    parser.add_argument(
        "--config", type=str, default="config.json", help="配置文件路径"
    )
    parser.add_argument("--simple", action="store_true", help="简单模式（不使用 LLM）")
    parser.add_argument("--dry-run", action="store_true", help="仅生成描述，不更新 PR")
    args = parser.parse_args()

    try:
        config = load_config(args.config)
    except Exception as e:
        print(f"❌ {e}")
        sys.exit(1)

    llm_api_key = None if args.simple else get_llm_api_key()

    print("=" * 60)
    print("🤖 LLM 驱动的语义化 PR 描述生成工具")
    print("=" * 60)
    print(f"PR 编号: #{args.pr}")
    print(f"仓库: {config['atomgit']['owner']}/{config['atomgit']['repo']}")
    print(f"模式: {'简单' if args.simple else '语义化'}")
    print(f"LLM 分析: {'启用' if llm_api_key else '禁用'}")
    print()

    api = AtomGitAPI.from_config(args.config)

    print("正在收集 PR 信息...")

    pr = api.get_pull_request(args.pr)
    commits = api.get_pr_commits(args.pr)
    files = api.get_pr_files(args.pr)

    source_branch = pr.get("head", {}).get("ref", "HEAD")

    print(f"✓ {len(commits)} 个 commits")
    print(f"✓ {len(files)} 个文件变更")
    print()

    print("正在生成描述...")
    changes = generate_semantic_changes(files, api, llm_api_key, source_branch)
    description = generate_pr_description(changes, files)

    print()
    print("=" * 60)
    print("生成的 PR 描述:")
    print("=" * 60)
    print(description)
    print("=" * 60)
    print()

    output_file = f"/tmp/atomgit_pr_{args.pr}_description.md"
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(description)
    print(f"✓ 描述已保存到: {output_file}")

    if args.dry_run:
        print("\n⚠ Dry run 模式，未更新 PR")
        return

    try:
        import readline

        answer = input("\n是否更新 PR 描述？(y/n): ").strip().lower()
    except:
        answer = "n"

    if answer in ("y", "yes"):
        print("正在更新 PR...")
        api.update_pull_request(args.pr, body=description)
        print(f"\n✅ PR 描述更新成功")
        print(f"PR 链接: {api.get_pr_url(args.pr)}")
    else:
        print("已取消更新")


if __name__ == "__main__":
    main()
