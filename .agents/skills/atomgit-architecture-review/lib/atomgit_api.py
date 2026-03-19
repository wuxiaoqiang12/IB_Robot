"""
AtomGit API 封装
提供 AtomGit/GitCode API 调用能力
"""

import os
import re
import json
import requests
from typing import Optional, Dict, List, Any
from dataclasses import dataclass
from urllib.parse import quote as url_quote


@dataclass
class AtomGitConfig:
    """AtomGit API 配置"""
    token: str
    owner: str
    repo: str
    base_url: str = "https://api.atomgit.com"


class AtomGitAPI:
    """AtomGit API 类"""
    
    def __init__(self, config: AtomGitConfig):
        self.config = config
        self.headers = {
            "Authorization": f"Bearer {config.token}",
            "Content-Type": "application/json",
            "User-Agent": "IB-Robot-Architecture-Review/1.0"
        }
    
    def request(self, endpoint: str, method: str = "GET", body: Optional[dict] = None) -> Any:
        """发送 HTTP 请求"""
        url = f"{self.config.base_url}{endpoint}"
        
        response = requests.request(
            method=method,
            url=url,
            headers=self.headers,
            json=body
        )
        
        if response.status_code not in (200, 201):
            raise Exception(f"API 请求失败: {response.status_code} - {response.text}")
        
        return response.json()
    
    def get_pull_requests(self, state: str = "open") -> List[dict]:
        """获取 PR 列表"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls?state={state}&per_page=100"
        )
    
    def get_pull_request(self, pr_number: int) -> dict:
        """获取单个 PR 详情"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}"
        )
    
    def get_pr_files(self, pr_number: int) -> List[dict]:
        """获取 PR 文件变更"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/files"
        )
    
    def get_pr_commits(self, pr_number: int) -> List[dict]:
        """获取 PR 的 commits 列表"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/commits"
        )
    
    def get_pr_diff(self, pr_number: int) -> Dict[str, dict]:
        """获取 PR diff"""
        files = self.get_pr_files(pr_number)
        diffs = {}
        
        for file in files:
            if file.get("patch"):
                patch_content = file["patch"].get("diff", file["patch"]) if isinstance(file["patch"], dict) else file["patch"]
                diffs[file["filename"]] = {
                    "patch": patch_content,
                    "additions": file.get("additions", 0),
                    "deletions": file.get("deletions", 0),
                    "status": file.get("status", "modified")
                }
        
        return diffs
    
    def calculate_position(self, patch: str, line_number: int, is_new_file: bool = False) -> Optional[int]:
        """计算行号在 diff 中的 position"""
        if not patch:
            return line_number if is_new_file else None
        
        if not line_number or line_number <= 0:
            return None
        
        lines = patch.split("\n")
        position = 0
        current_new_line = 0
        in_hunk = False
        
        for i, line in enumerate(lines):
            hunk_match = re.match(r"^@@\s+-\d+,?\d*\s+\+(\d+),?\d*\s+@@", line)
            if hunk_match:
                in_hunk = True
                position = i + 1
                continue
            
            if not in_hunk:
                continue
            
            first_char = line[0] if line else ""
            
            if first_char == "+":
                current_new_line += 1
                if current_new_line == line_number:
                    return position
            elif first_char == " ":
                current_new_line += 1
                if current_new_line == line_number:
                    return position
            
            position += 1
        
        if is_new_file:
            return line_number
        
        return None
    
    def get_file_content(self, file_path: str, ref: str = "HEAD") -> str:
        """获取文件内容"""
        encoded_path = url_quote(file_path, safe="")
        data = self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/contents/{encoded_path}?ref={ref}"
        )
        
        if data.get("content"):
            import base64
            return base64.b64decode(data["content"]).decode("utf-8")
        return ""
    
    def submit_inline_comment(self, pr_number: int, comment: dict) -> dict:
        """提交行内评论"""
        if not comment.get("path"):
            raise Exception("Cannot submit inline comment without path")
        
        payload = {
            "body": comment["body"],
            "path": comment["path"]
        }
        
        if comment.get("position") is not None:
            payload["position"] = comment["position"]
            if comment.get("commitId"):
                payload["commit_id"] = comment["commitId"]
        elif comment.get("line"):
            payload["position"] = comment["line"]
            if comment.get("commitId"):
                payload["commit_id"] = comment["commitId"]
        else:
            raise Exception(f"Cannot submit inline comment for {comment['path']}: no position or line provided")
        
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/comments",
            method="POST",
            body=payload
        )
    
    def submit_pr_comment(self, pr_number: int, body: str) -> dict:
        """提交 PR 整体评论"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/comments",
            method="POST",
            body={"body": body}
        )
    
    def submit_batch_comments(self, pr_number: int, comments: List[dict]) -> List[dict]:
        """批量提交评论"""
        results = []
        comment_base_url = f"https://atomgit.com/{self.config.owner}/{self.config.repo}/pulls/{pr_number}"
        
        for comment in comments:
            try:
                result = self.submit_inline_comment(pr_number, comment)
                comment_url = f"{comment_base_url}#comment-{result.get('id', '')}" if result.get("id") else comment_base_url
                results.append({
                    "success": True,
                    "comment": comment,
                    "result": result,
                    "comment_url": comment_url
                })
            except Exception as e:
                results.append({
                    "success": False,
                    "comment": comment,
                    "error": str(e),
                    "comment_url": None
                })
        
        return results
    
    def get_pr_url(self, pr_number: int) -> str:
        """获取 PR 的 URL"""
        return f"https://atomgit.com/{self.config.owner}/{self.config.repo}/pull/{pr_number}"
    
    @staticmethod
    def parse_atomgit_url(url: str) -> dict:
        """从 AtomGit URL 解析仓库和分支信息"""
        patterns = [
            r"atomgit\.com/([^/]+)/([^/]+)/(tree|commits)/([^/]+)",
            r"atomgit\.com/([^/]+)/([^/]+)"
        ]
        
        for pattern in patterns:
            match = re.search(pattern, url)
            if match:
                owner = match.group(1)
                repo = match.group(2).replace(".git", "")
                branch = match.group(4) if len(match.groups()) > 3 else "master"
                return {"owner": owner, "repo": repo, "branch": branch}
        
        raise Exception(f"无法解析 AtomGit URL: {url}")
    
    @staticmethod
    def from_config(config_path: str = "config.json") -> "AtomGitAPI":
        """从配置文件创建 API 实例"""
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        
        atomgit_config = AtomGitConfig(
            token=config["atomgit"]["token"],
            owner=config["atomgit"]["owner"],
            repo=config["atomgit"]["repo"],
            base_url=config["atomgit"].get("baseUrl", "https://api.atomgit.com")
        )
        
        return AtomGitAPI(atomgit_config)
