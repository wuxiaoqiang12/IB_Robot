#!/usr/bin/env python3
"""
AtomGit Pull Request Creator

自动创建 PR，从当前分支生成 PR 标题和描述

用法：
    python3 create_pr.py --branch <branch> [--title "标题"] [--dry-run]
"""

import os
import sys
import json
import argparse
import subprocess
from pathlib import Path
from typing import List, Dict, Optional
from urllib.parse import quote as url_quote

sys.path.insert(0, str(Path(__file__).parent.parent / "lib"))

from atomgit_api import AtomGitAPI, AtomGitConfig


def run_git(args: List[str], cwd: str = None) -> str:
    """运行 git 命令并返回输出"""
    result = subprocess.run(
        ["git"] + args, cwd=cwd or os.getcwd(), capture_output=True, text=True
    )
    if result.returncode != 0:
        raise Exception(f"Git 命令失败: {' '.join(args)}\n{result.stderr}")
    return result.stdout.strip()


def get_current_branch() -> str:
    """获取当前分支名"""
    return run_git(["rev-parse", "--abbrev-ref", "HEAD"])


def is_valid_branch_name(name: str) -> bool:
    """验证分支名是否有效"""
    if not name or name in ("master", "main", "HEAD"):
        return False
    try:
        run_git(["rev-parse", "--verify", name])
        return True
    except:
        return False


def get_commit_messages(branch: str, base_branch: str = "master") -> List[Dict]:
    """获取分支相对于基线的提交信息"""
    try:
        log_format = "%H%n%s%n%b%n---COMMIT_END---"
        output = run_git(["log", f"{base_branch}..{branch}", f"--format={log_format}"])
    except:
        output = run_git(
            ["log", f"origin/{base_branch}..{branch}", f"--format={log_format}"]
        )

    commits = []
    for block in output.split("---COMMIT_END---"):
        if not block.strip():
            continue
        lines = block.strip().split("\n", 2)
        if len(lines) >= 2:
            commits.append(
                {
                    "hash": lines[0],
                    "subject": lines[1],
                    "body": lines[2] if len(lines) > 2 else "",
                }
            )

    return commits


def get_changed_files(branch: str, base_branch: str = "master") -> List[str]:
    """获取变更文件列表"""
    try:
        output = run_git(["diff", "--name-only", f"{base_branch}...{branch}"])
    except:
        output = run_git(["diff", "--name-only", f"origin/{base_branch}...{branch}"])
    return [f for f in output.split("\n") if f.strip()]


def get_diff_stats(branch: str, base_branch: str = "master") -> Dict:
    """获取 diff 统计信息"""
    try:
        output = run_git(["diff", "--stat", f"{base_branch}...{branch}"])
    except:
        output = run_git(["diff", "--stat", f"origin/{base_branch}...{branch}"])

    stats = {"files_changed": 0, "insertions": 0, "deletions": 0}

    for line in output.split("\n"):
        if "file" in line and "changed" in line:
            parts = line.split(",")
            for part in parts:
                part = part.strip()
                if "file" in part:
                    stats["files_changed"] = int(part.split()[0])
                elif "insertion" in part:
                    stats["insertions"] = int(part.split()[0])
                elif "deletion" in part:
                    stats["deletions"] = int(part.split()[0])

    return stats


def generate_pr_description(
    branch: str,
    commits: List[Dict],
    files: List[str],
    stats: Dict,
    base_branch: str = "master",
) -> str:
    """生成 PR 描述"""
    lines = []

    lines.append("## Summary")
    lines.append("")

    if commits:
        lines.append("### Commits")
        for c in commits:
            lines.append(f"- {c['subject']}")
        lines.append("")

    lines.append("### Changed Files")
    lines.append(f"- Files changed: {stats['files_changed']}")
    lines.append(f"- Insertions: +{stats['insertions']}")
    lines.append(f"- Deletions: -{stats['deletions']}")
    lines.append("")

    if files:
        lines.append("### File List")
        for f in files[:20]:
            lines.append(f"- `{f}`")
        if len(files) > 20:
            lines.append(f"- ... and {len(files) - 20} more files")
        lines.append("")

    lines.append("---")
    lines.append("")
    lines.append("## How to Test")
    lines.append("- Verify code compiles/builds successfully")
    lines.append("- Run relevant tests")

    return "\n".join(lines)


def load_config(config_path: str) -> dict:
    """加载配置文件"""
    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    if not config.get("atomgit") or not config["atomgit"].get("token"):
        raise Exception("配置文件中缺少 atomgit.token")

    return config


def main():
    parser = argparse.ArgumentParser(description="AtomGit PR 创建工具")
    parser.add_argument("--branch", type=str, help="源分支名")
    parser.add_argument("--base", type=str, default="master", help="目标分支名")
    parser.add_argument("--title", type=str, help="PR 标题")
    parser.add_argument(
        "--config", type=str, default="config.json", help="配置文件路径"
    )
    parser.add_argument("--dry-run", action="store_true", help="仅显示计划，不实际创建")
    args = parser.parse_args()

    try:
        config = load_config(args.config)
    except Exception as e:
        print(f"❌ 加载配置失败: {e}")
        sys.exit(1)

    branch = args.branch or get_current_branch()

    if not is_valid_branch_name(branch):
        print(f"❌ 无效的分支名: {branch}")
        sys.exit(1)

    if branch in ("master", "main"):
        print("❌ 不能从 master/main 分支创建 PR")
        sys.exit(1)

    print("=" * 60)
    print("🚀 AtomGit PR Creator")
    print("=" * 60)
    print(f"源分支: {branch}")
    print(f"目标分支: {args.base}")
    print(f"仓库: {config['atomgit']['owner']}/{config['atomgit']['repo']}")
    print()

    print(">>> 获取提交信息...")
    commits = get_commit_messages(branch, args.base)
    print(f"✓ 找到 {len(commits)} 个提交")

    if not commits:
        print(f"❌ 在 {args.base} 和 {branch} 之间未找到提交")
        print("   请确保你的分支与 origin/master 同步")
        sys.exit(1)

    print(">>> 获取变更文件...")
    files = get_changed_files(branch, args.base)
    print(f"✓ 找到 {len(files)} 个变更文件")

    print(">>> 计算变更统计...")
    stats = get_diff_stats(branch, args.base)
    print(
        f"✓ +{stats['insertions']}/-{stats['deletions']} 行, {stats['files_changed']} 文件"
    )

    print(">>> 生成 PR 描述...")
    description = generate_pr_description(branch, commits, files, stats, args.base)

    title = args.title or commits[0]["subject"]
    print(f"✓ 标题: {title}")

    print()
    print("=" * 60)
    print("生成的 PR 描述:")
    print("=" * 60)
    print(description)
    print("=" * 60)
    print()

    if args.dry_run:
        print("⚠ Dry run 模式，未创建 PR")
        sys.exit(0)

    try:
        answer = input("\n是否创建 PR？(y/n): ").strip().lower()
    except:
        answer = "n"

    if answer not in ("y", "yes"):
        print("已取消创建")
        return

    print(">>> 创建 PR...")
    api = AtomGitAPI.from_config(args.config)

    try:
        pr = api.create_pull_request(
            title=title, body=description, head=branch, base=args.base
        )
        pr_number = pr.get("number")
        pr_url = api.get_pr_url(pr_number)

        print()
        print("✅ PR 创建成功!")
        print(f"PR 编号: #{pr_number}")
        print(f"PR 链接: {pr_url}")

    except Exception as e:
        print(f"❌ 创建 PR 失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
