#!/usr/bin/env python3
"""
AtomGit 代码审查脚本
功能: 提取 PR 变更信息，支持提交行内评论

使用方式:
1. 提取 PR 信息: python3 atomgit_reviewer.py --pr 123
2. 从 JSON 提交评论: python3 atomgit_reviewer.py --pr 123 --issues-from-json issues.json
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Optional

sys.path.insert(0, str(Path(__file__).parent.parent / "lib"))

from atomgit_api import AtomGitAPI, AtomGitConfig
from comment_formatter import CommentFormatter, CodeIssue


class CodeReviewer:
    """代码审查器"""

    def __init__(self, api: AtomGitAPI, formatter: CommentFormatter):
        self.api = api
        self.formatter = formatter

    def extract_pr_info(self, pr_number: int) -> dict:
        """提取 PR 信息"""
        pr = self.api.get_pull_request(pr_number)
        files = self.api.get_pr_files(pr_number)

        changed_files = []
        for f in files:
            if f.get("status") != "removed":
                changed_files.append(
                    {
                        "filename": f.get("filename"),
                        "status": f.get("status"),
                        "additions": f.get("additions", 0),
                        "deletions": f.get("deletions", 0),
                        "patch": f.get("patch"),
                    }
                )

        return {
            "pr": {
                "number": pr.get("number"),
                "title": pr.get("title"),
                "author": pr.get("user", {}).get("login"),
                "branch": f"{pr.get('head', {}).get('ref')} → {pr.get('base', {}).get('ref')}",
                "head_sha": pr.get("head", {}).get("sha"),
                "changed_files": changed_files,
            }
        }

    def load_issues_from_json(self, json_path: str) -> List[CodeIssue]:
        """从 JSON 文件加载问题"""
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        issues = []
        for item in data:
            issue = CodeIssue(
                file=item.get("file", ""),
                line=item.get("line", 0),
                type=item.get("type", "bug"),
                severity=item.get("severity", "warning"),
                confidence=item.get("confidence", 80),
                title=item.get("title", ""),
                description=item.get("description", ""),
                context_code=item.get("contextCode") or item.get("context_code"),
                fix_code=item.get("fix", {}).get("code")
                if isinstance(item.get("fix"), dict)
                else item.get("fix_code"),
                fix_explanation=item.get("fix", {}).get("explanation")
                if isinstance(item.get("fix"), dict)
                else item.get("fix_explanation"),
            )
            issues.append(issue)

        return issues

    def submit_issues(self, pr_number: int, issues: List[CodeIssue]) -> Dict:
        """提交问题到 PR"""
        pr = self.api.get_pull_request(pr_number)
        diffs = self.api.get_pr_diff(pr_number)

        issues = self.formatter.deduplicate(issues)

        positions = {}
        for issue in issues:
            if issue.file not in positions:
                diff_info = diffs.get(issue.file, {})
                is_new_file = diff_info.get("status") == "added"
                patch = diff_info.get("patch", "")
                positions[issue.file] = {}

            diff_info = diffs.get(issue.file, {})
            is_new_file = diff_info.get("status") == "added"
            patch = diff_info.get("patch", "")
            position = self.api.calculate_position(patch, issue.line, is_new_file)
            if position is not None:
                positions[issue.file][issue.line] = position

        comments = self.formatter.format_issues(issues, positions)

        summary = self.formatter.format_summary(issues, pr_number, pr.get("title", ""))
        self.api.submit_pr_comment(pr_number, summary)
        print(f"✅ 已提交摘要评论\n")

        if comments:
            results = self.api.submit_batch_comments(pr_number, comments)
            success_count = sum(1 for r in results if r["success"])

            print(f"✅ 提交 {success_count}/{len(results)} 条评论\n")

            for result in results:
                if result["success"]:
                    print(f"  ✅ {result['comment']['path']} → {result['comment_url']}")
                else:
                    print(f"  ❌ {result['comment']['path']} - {result['error']}")
        else:
            print("⚠️  没有符合条件的问题需要提交\n")

        return {
            "total_issues": len(issues),
            "submitted_comments": len(comments),
            "summary_submitted": True,
        }


def main():
    parser = argparse.ArgumentParser(description="AtomGit 代码审查")
    parser.add_argument("--pr", type=int, required=True, help="PR 编号")
    parser.add_argument(
        "--config", type=str, default="config.json", help="配置文件路径"
    )
    parser.add_argument(
        "--issues-from-json", type=str, help="从 JSON 文件加载问题并提交"
    )
    parser.add_argument("--extract-only", action="store_true", help="仅提取 PR 信息")
    parser.add_argument(
        "--output-dir", type=str, default="./review-reports", help="输出目录"
    )
    parser.add_argument("--threshold", type=int, default=80, help="置信度阈值")
    parser.add_argument("--dry-run", action="store_true", help="仅显示计划，不提交")
    args = parser.parse_args()

    api = AtomGitAPI.from_config(args.config)
    formatter = CommentFormatter(confidence_threshold=args.threshold)
    reviewer = CodeReviewer(api, formatter)

    print(f"🔍 AtomGit 代码审查工具\n")
    print(f"{'=' * 50}")
    print(f"📋 PR #{args.pr}")
    print(f"{'=' * 50}\n")

    if args.issues_from_json:
        print(f"📂 从 JSON 加载问题: {args.issues_from_json}\n")

        issues = reviewer.load_issues_from_json(args.issues_from_json)
        print(f"📝 加载了 {len(issues)} 个问题\n")

        if args.dry_run:
            print("ℹ️  Dry run 模式：将显示提交计划但不执行\n")
            for issue in issues:
                if issue.confidence >= args.threshold:
                    print(
                        f"  - {issue.file}:{issue.line} [{issue.severity}] {issue.title}"
                    )
            print("")
            return

        result = reviewer.submit_issues(args.pr, issues)

        print(f"\n{'=' * 50}")
        print(f"✅ 审查完成")
        print(f"{'=' * 50}\n")
        print(f"📊 统计:")
        print(f"   总问题数: {result['total_issues']}")
        print(f"   提交评论数: {result['submitted_comments']}")
        print(f"\n🔗 PR 链接: {api.get_pr_url(args.pr)}\n")

    else:
        print("📂 提取 PR 信息...\n")

        pr_info = reviewer.extract_pr_info(args.pr)

        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        output_file = output_dir / f"pr-{args.pr}-info.json"
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(pr_info, f, indent=2, ensure_ascii=False)

        print(f"📄 PR 信息已保存: {output_file}\n")

        print(f"📊 变更摘要:")
        print(f"   标题: {pr_info['pr']['title']}")
        print(f"   作者: {pr_info['pr']['author']}")
        print(f"   分支: {pr_info['pr']['branch']}")
        print(f"   文件: {len(pr_info['pr']['changed_files'])} 个\n")

        print("💡 使用以下命令提交审查结果:")
        print(
            f"   python3 .agents/skills/atomgit-code-review/scripts/atomgit_reviewer.py --pr {args.pr} --issues-from-json issues.json\n"
        )


if __name__ == "__main__":
    main()
