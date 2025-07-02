#!/usr/bin/env python3
import os, shutil, subprocess, argparse, xml.etree.ElementTree as ET
from pathlib import Path
from git import Repo, GitCommandError
import sys

def find_ros_packages(base_path):
    """Find ROS packages and return dict{path:name}"""
    packages = {}

    for root, dirs, files in os.walk(base_path):
        if "package.xml" in files:
            try:
                name = ET.parse(Path(root) / "package.xml").findtext("name")
                packages[root] = name
                dirs.clear()
            except ET.ParseError:
                print(f"⚠️ Failed to parse {root}/package.xml")
    return packages

def clear_except(path, keep=[".git", "debian"]):
    for item in Path(path).iterdir():
        if item.name not in keep:
            shutil.rmtree(item) if item.is_dir() else item.unlink()
            print(f"Remove {item}")

def copy_package(src, dst):
    for root, dirs, files in os.walk(src):
        dirs[:] = [d for d in dirs if d not in ['.git', '.github']]
        for f in files:
            s = Path(root) / f
            d = Path(dst) / Path(root).relative_to(src) / f
            d.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(s, d)
            print(f"Copy {s} to {d}")

def create_pr(base, head, msg):
    try:
        subprocess.run(["gh", "pr", "create", "--base", base, "--head", head,
                        "--title", f"Sync {base}", "--body", msg],
                       check=True, text=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ PR creation failed: {e.stderr}")

def sync(repo, base_branch, pkg_path, pkg_name, mode):
    target_branch = f"debian/jazzy/noble/{pkg_name}"
    worktree = f"worktree_{pkg_name}"
    try:
        repo.git.worktree("add", worktree, target_branch)
        if not (Path(worktree) / "debian").exists():
            print(f"::warning file=sync_debian_branches.py,line=48,title=Missing debian directory::❗ 'debian' directory missing in branch {target_branch}.")

        print("::group::Start file operation.")

        clear_except(worktree)
        repo.git.checkout(base_branch)
        copy_package(pkg_path, worktree)

        print("::endgroup::")

        wrepo = Repo(worktree)
        wrepo.git.add(A=True)
        if wrepo.is_dirty():
            commit_hash = repo.head.commit.hexsha
            msg = f"sync from {base_branch}: for {pkg_name}\nSource commit: {commit_hash}"
            wrepo.index.commit(msg)
            if mode == "direct":
                wrepo.git.push("origin", target_branch)
            else:
                temp = f"sync-{pkg_name}"
                wrepo.git.checkout("-b", temp)
                wrepo.git.push("origin", temp, force=True)
                create_pr(target_branch, temp, msg)
            print(f"🚩 {pkg_name} updated!")
        else:
            print(f"⏭️ No changes for {pkg_name}")
    except GitCommandError as e:
        print(f"❌ Sync failed: {str(e)}")
    finally:
        repo.git.worktree("remove", worktree, "--force")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["pr", "direct"], default="pr")
    parser.add_argument("--path", default=".")
    args = parser.parse_args()

    repo = Repo(args.path)
    packages = find_ros_packages(args.path)
    print(f"📦 Found {len(packages)} packages: {packages}")

    for path, name in packages.items():
        print(f"🔄 Syncing {name}")
        sync(repo, "main", path, name, args.mode)

if __name__ == "__main__":
    main()
