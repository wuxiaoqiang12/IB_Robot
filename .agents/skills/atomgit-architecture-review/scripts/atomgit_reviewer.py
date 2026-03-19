#!/usr/bin/env python3
"""
AtomGit 架构审查脚本
对 PR 进行 IB_Robot 架构合规性审查
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Optional

# 添加 lib 目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent / "lib"))

from atomgit_api import AtomGitAPI, AtomGitConfig
from comment_formatter import CommentFormatter, ArchitectureIssue


class ArchitectureReviewer:
    """架构审查器"""
    
    def __init__(self, api: AtomGitAPI):
        self.api = api
        self.formatter = CommentFormatter()
    
    def check_config_driven(self, file_path: str, content: str, diff: str) -> List[ArchitectureIssue]:
        """检查配置驱动原则"""
        issues = []
        
        # 检查硬编码阈值
        import re
        
        # 常见硬编码模式
        patterns = [
            (r"(\w+_threshold\s*=\s*\d+)", "配置硬编码阈值"),
            (r"(\w+_timeout\s*=\s*\d+)", "配置硬编码超时"),
            (r"(\w+_rate\s*=\s*\d+)", "配置硬编码频率"),
            (r"(\w+_count\s*=\s*\d+)", "配置硬编码计数"),
            (r"(joint_names\s*=\s*\[)", "硬编码关节名称"),
            (r"(device_path\s*=\s*['\"])", "硬编码设备路径"),
        ]
        
        for i, line in enumerate(content.split("\n"), 1):
            for pattern, desc in patterns:
                match = re.search(pattern, line)
                if match:
                    # 检查是否在注释中
                    if not line.strip().startswith("#"):
                        issues.append(ArchitectureIssue(
                            file=file_path,
                            line=i,
                            title=desc,
                            description=f"在代码中发现: `{match.group(1)}`",
                            severity="warning",
                            pillar="ssot",
                            fix=f"考虑将此配置项提取到 robot_config YAML 文件中",
                            context_code=line.strip()
                        ))
        
        return issues
    
    def check_ros2_native(self, file_path: str, content: str, diff: str) -> List[ArchitectureIssue]:
        """检查 ROS 2 原生性"""
        issues = []
        
        import re
        
        # 检查非 ROS 2 方式的节点通信
        patterns = [
            (r"(import\s+socket\s*$)", "使用原生 socket 而非 ROS 2 通信"),
            (r"(import\s+subprocess\s*$)", "使用 subprocess 而非 ROS 2 服务"),
            (r"(import\s+http\.client)", "使用 HTTP 而非 ROS 2 服务"),
            (r"(open\s*\(\s*['\"/dev/)", "直接访问设备文件而非 ros2_control"),
        ]
        
        for i, line in enumerate(content.split("\n"), 1):
            for pattern, desc in patterns:
                match = re.search(pattern, line)
                if match:
                    issues.append(ArchitectureIssue(
                        file=file_path,
                        line=i,
                        title=desc,
                        description=f"在代码中发现: `{match.group(1)}`",
                        severity="warning",
                        pillar="ros2",
                        fix="考虑使用 ROS 2 原生的通信机制（topic/service/action）",
                        context_code=line.strip()
                    ))
        
        return issues
    
    def check_contract_compliance(self, file_path: str, content: str, diff: str) -> List[ArchitectureIssue]:
        """检查契约合规性"""
        issues = []
        
        # 检查是否正确使用 Contract
        import re
        
        # 检查是否直接操作 tensor 而不通过 tensormsg
        if "tensor" in content.lower() and "ros" in content.lower():
            if "tensormsg" not in content.lower():
                # 查找具体行
                for i, line in enumerate(content.split("\n"), 1):
                    if "tensor" in line.lower() and ("ros" in line.lower() or "msg" in line.lower()):
                        issues.append(ArchitectureIssue(
                            file=file_path,
                            line=i,
                            title="跨域数据转换缺失 tensormsg",
                            description="在 ROS 和 tensor 之间直接转换，应使用 tensormsg 模块",
                            severity="warning",
                            pillar="tensormsg",
                            fix="使用 tensormsg 模块进行 ROS 消息与 tensor 之间的转换",
                            context_code=line.strip()
                        ))
                        break
        
        return issues
    
    def check_python_style(self, file_path: str, content: str, diff: str) -> List[ArchitectureIssue]:
        """检查 Python 代码风格"""
        issues = []
        
        import re
        
        # 检查常见问题
        for i, line in enumerate(content.split("\n"), 1):
            # 检查 print 语句
            if re.search(r"\bprint\s*\(", line) and not line.strip().startswith("#"):
                # 排除测试文件
                if "test" not in file_path.lower():
                    issues.append(ArchitectureIssue(
                        file=file_path,
                        line=i,
                        title="使用 print 而非日志",
                        description="生产代码应使用 logging 而非 print",
                        severity="suggestion",
                        pillar="python",
                        fix="使用 `import logging` 和 `logger.info()` 替代 print",
                        context_code=line.strip()
                    ))
            
            # 检查 TODO/FIXME
            if re.search(r"#\s*(TODO|FIXME)", line, re.IGNORECASE):
                issues.append(ArchitectureIssue(
                    file=file_path,
                    line=i,
                    title="未完成的代码",
                    description="代码中存在 TODO/FIXME 注释",
                    severity="info",
                    pillar="python",
                    fix="完成或移除 TODO/FIXME 注释",
                    context_code=line.strip()
                ))
        
        return issues
    
    def review_pr(self, pr_number: int) -> List[ArchitectureIssue]:
        """审查 PR"""
        all_issues = []
        
        # 获取 PR 文件
        files = self.api.get_pr_files(pr_number)
        
        for file_info in files:
            file_path = file_info["filename"]
            
            # 只审查 Python 文件
            if not file_path.endswith(".py"):
                continue
            
            # 跳过测试文件
            if "test" in file_path.lower():
                continue
            
            # 跳过 __init__.py
            if file_path.endswith("__init__.py"):
                continue
            
            try:
                # 获取文件内容
                content = self.api.get_file_content(file_path, f"pull/{pr_number}/head")
                diff = file_info.get("patch", {}).get("diff", "") if isinstance(file_info.get("patch"), dict) else file_info.get("patch", "")
                
                # 执行各种检查
                all_issues.extend(self.check_config_driven(file_path, content, diff))
                all_issues.extend(self.check_ros2_native(file_path, content, diff))
                all_issues.extend(self.check_contract_compliance(file_path, content, diff))
                all_issues.extend(self.check_python_style(file_path, content, diff))
                
            except Exception as e:
                print(f"Warning: Could not review {file_path}: {e}")
        
        return all_issues
    
    def submit_review(self, pr_number: int, issues: List[ArchitectureIssue]) -> None:
        """提交审查结果"""
        if not issues:
            # 提交通过评论
            summary = self.formatter.format_summary(issues)
            self.api.submit_pr_comment(pr_number, summary)
            print(f"✅ 提交架构审查通过评论到 PR #{pr_number}")
        else:
            # 提交行内评论
            comments = self.formatter.format_issues(issues)
            results = self.api.submit_batch_comments(pr_number, comments)
            
            success_count = sum(1 for r in results if r["success"])
            print(f"✅ 提交 {success_count}/{len(results)} 条架构评论到 PR #{pr_number}")
            
            for result in results:
                if not result["success"]:
                    print(f"  ❌ 失败: {result['comment']['path']} - {result['error']}")


def main():
    parser = argparse.ArgumentParser(description="AtomGit 架构审查")
    parser.add_argument("--pr", type=int, required=True, help="PR 编号")
    parser.add_argument("--config", type=str, default="config.json", help="配置文件路径")
    parser.add_argument("--dry-run", action="store_true", help="仅生成报告，不提交评论")
    args = parser.parse_args()
    
    # 加载 API
    api = AtomGitAPI.from_config(args.config)
    
    # 创建审查器
    reviewer = ArchitectureReviewer(api)
    
    print(f"🔍 开始审查 PR #{args.pr}...")
    
    # 执行审查
    issues = reviewer.review_pr(args.pr)
    
    print(f"\n发现 {len(issues)} 个架构问题")
    
    # 生成报告
    summary = reviewer.formatter.format_summary(issues)
    print("\n" + "=" * 50)
    print(summary)
    
    if not args.dry_run:
        # 提交评论
        reviewer.submit_review(args.pr, issues)
    else:
        print("\n⚠️  Dry run 模式，未提交评论")


if __name__ == "__main__":
    main()
