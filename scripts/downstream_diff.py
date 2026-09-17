#!/usr/bin/env python3
"""downstream_diff.py —— 校验下游仓库里的 ctlkit 副本与上游逐字一致（忽略来源戳行）。

用法：
    python3 scripts/downstream_diff.py <下游仓库路径>            # 用本仓库作上游
    python3 scripts/downstream_diff.py <下游仓库路径> --lib <路径>

判据（对应 roadmap D-6 的“批次迁移后跑下游 diff 校验”）：
  · 上游每个文件（inc/ctl/*.hpp + src/*.cpp）在下游都能找到同名文件；
  · 内容逐行一致，只允许差异在「来源戳行」（形如 `// from ctlkit vX.Y.Z`）。
退出码：0 = 全部一致；1 = 有缺失或差异（打印清单）。
"""

import argparse
import pathlib
import re
import sys

STAMP = re.compile(r"^\s*//\s*from ctlkit\b")
SKIP_DIRS = {".git", "build", "cmake-build-debug", "cmake-build-release", "__pycache__", ".idea"}


def norm_lines(path: pathlib.Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    return [ln.rstrip() for ln in text.splitlines() if not STAMP.match(ln)]


def library_files(lib: pathlib.Path):
    files = sorted((lib / "inc" / "ctl").glob("*.hpp")) + sorted((lib / "src").glob("*.cpp"))
    return {p.name: p for p in files}


def find_downstream(target: pathlib.Path, names):
    """在下游递归找同名文件（跳过构建产物目录），返回 {name: [paths]}"""
    hits = {n: [] for n in names}
    for p in target.rglob("*"):
        if not p.is_file() or p.name not in hits:
            continue
        if any(part in SKIP_DIRS for part in p.relative_to(target).parts):
            continue
        hits[p.name].append(p)
    return hits


def main() -> int:
    ap = argparse.ArgumentParser(description="校验下游 ctlkit 副本与上游一致（忽略来源戳）")
    ap.add_argument("target", help="下游仓库路径")
    ap.add_argument("--lib", default=str(pathlib.Path(__file__).resolve().parent.parent),
                    help="上游 ctlkit 路径（默认本脚本所在仓库）")
    args = ap.parse_args()

    lib = pathlib.Path(args.lib).resolve()
    target = pathlib.Path(args.target).resolve()
    if not (lib / "inc" / "ctl").is_dir():
        sys.exit(f"❌ 上游路径不对（找不到 inc/ctl）：{lib}")
    if not target.is_dir():
        sys.exit(f"❌ 下游路径不存在：{target}")

    libs = library_files(lib)
    hits = find_downstream(target, libs.keys())

    bad, missing, ok = [], [], []
    for name, src in libs.items():
        found = hits[name]
        if not found:
            missing.append(name)
            continue
        s_lines = norm_lines(src)
        for dst in found:
            if norm_lines(dst) == s_lines:
                ok.append((name, dst.relative_to(target).as_posix()))
            else:
                bad.append((name, dst.relative_to(target).as_posix()))

    print(f"上游 {lib}")
    print(f"下游 {target}   文件 {len(libs)} 个\n")
    for name, rel in ok:
        print(f"  ✅ identical  {rel}")
    for name, rel in bad:
        print(f"  ❌ differs    {rel}   ← 与上游 {name} 不一致（除来源戳外还有差异）")
    for name in missing:
        print(f"  ⚠️  缺失       {name}   ← 下游没有这个文件（迁移未完成？）")

    print()
    if bad or missing:
        print(f"❌ 不一致 {len(bad)} 处 · 缺失 {len(missing)} 个 —— 迁移未通过")
        return 1
    print(f"✅ 全部一致（{len(ok)} 处匹配，已忽略来源戳行）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
