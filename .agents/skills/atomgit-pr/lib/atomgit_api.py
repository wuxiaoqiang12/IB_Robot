"""
AtomGit API 封装
提供 AtomGit API 调用能力
"""

import os
import re
import json
import base64
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
            "User-Agent": "AtomGit-PR-Skill/1.0",
        }

    def request(
        self, endpoint: str, method: str = "GET", body: Optional[dict] = None
    ) -> Any:
        """发送 HTTP 请求"""
        url = f"{self.config.base_url}{endpoint}"

        response = requests.request(
            method=method, url=url, headers=self.headers, json=body
        )

        if response.status_code not in (200, 201):
            raise Exception(f"API 请求失败: {response.status_code} - {response.text}")

        try:
            return response.json()
        except:
            return {"data": response.text}

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

    def get_pr_comments(self, pr_number: int) -> List[dict]:
        """获取 PR 评论列表"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/comments"
        )

    def get_file_content(self, file_path: str, ref: str = "HEAD") -> str:
        """获取文件内容"""
        encoded_path = url_quote(file_path, safe="")
        data = self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/contents/{encoded_path}?ref={ref}"
        )

        if data.get("content"):
            return base64.b64decode(data["content"]).decode("utf-8")
        return ""

    def create_pull_request(
        self,
        title: str,
        body: str,
        head: str,
        base: str = "master",
        draft: bool = False,
    ) -> dict:
        """创建 Pull Request"""
        if not title or not head or not base:
            raise Exception("创建 PR 需要 title, head, base 参数")

        final_head = head if ":" in head else f"{self.config.owner}:{head}"

        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls",
            method="POST",
            body={
                "title": title,
                "body": body or "",
                "head": final_head,
                "base": base,
                "draft": draft,
            },
        )

    def update_pull_request(
        self, pr_number: int, title: str = None, body: str = None, state: str = None
    ) -> dict:
        """更新 Pull Request"""
        payload = {}
        if title is not None:
            payload["title"] = title
        if body is not None:
            payload["body"] = body
        if state is not None:
            payload["state"] = state

        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}",
            method="PATCH",
            body=payload,
        )

    def submit_pr_comment(self, pr_number: int, body: str) -> dict:
        """提交 PR 整体评论"""
        return self.request(
            f"/api/v5/repos/{self.config.owner}/{self.config.repo}/pulls/{pr_number}/comments",
            method="POST",
            body={"body": body},
        )

    def get_pr_url(self, pr_number: int) -> str:
        """获取 PR 的 URL"""
        return f"https://atomgit.com/{self.config.owner}/{self.config.repo}/pull/{pr_number}"

    @staticmethod
    def parse_atomgit_url(url: str) -> dict:
        """从 AtomGit URL 解析仓库和分支信息"""
        patterns = [
            r"atomgit\.com/([^/]+)/([^/]+)/(tree|commits)/([^/]+)",
            r"atomgit\.com/([^/]+)/([^/]+)",
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
            base_url=config["atomgit"].get("baseUrl", "https://api.atomgit.com"),
        )

        return AtomGitAPI(atomgit_config)
