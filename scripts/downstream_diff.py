#!/usr/bin/env python3
"""downstream_diff.py —— 校验下游仓库里的 ctlkit 副本与上游逐字一致。

用法：
    python3 scripts/downstream_diff.py <下游仓库路径>                 # 自动探测布局
    python3 scripts/downstream_diff.py <下游仓库路径> --vendor third_party/ctlkit
    python3 scripts/downstream_diff.py <下游仓库路径> --selftest       # 跑工具自测（不碰下游）
    python3 scripts/downstream_diff.py <下游仓库路径> --lib <上游路径>

两种下游布局（对应 migration_WORK.md §2.1）：

  ① **vendor 整目录（推荐）**：`<下游>/third_party/ctlkit/{inc,src}` 逐字拷上游。
     比对**只在该目录**内按相对路径逐个文件进行；下游其它地方的**同名文件**必须二选一：
       · 删掉（不再需要）；或
       · 改成「转发头」——文件里带标记行 `// ctlkit-forwarder`，且确实 `#include "ctl/…"`。
       未标记的同名文件一律判失败（防止「旧副本留在原地各改各的」再次分叉）。

  ② **散拷贝**（旧布局）：全树按文件名找同名文件逐一比对（允许来源戳行）。

判据：
  · 上游每个文件在下游都能找到；
  · 逐行一致，只允许差异在「来源戳行」（形如 `// from ctlkit vX.Y.Z`）；
  · 转发头必须带标记行 + 至少一条 `#include "ctl/…"`。
退出码：0 = 通过；1 = 有缺失 / 差异 / 不合格的转发头 / 未标记的同名文件。
"""

import argparse
import pathlib
import re
import shutil
import sys
import tempfile

STAMP = re.compile(r"^\s*//\s*from ctlkit\b")
FORWARDER = re.compile(r"^\s*//.*ctlkit-forwarder")
# 转发头必须真的转发：出现 include "ctl/xxx.hpp"（本文件自己的 include）
CTL_INCLUDE = re.compile(r'^\s*#\s*include\s+"ctl/[^"]+"')
SKIP_DIRS = {".git", "build", "cmake-build-debug", "cmake-build-release", "__pycache__", ".idea"}
VENDOR_CANDIDATES = ("third_party/ctlkit", "vendor/ctlkit", "third_party/ctl-kit")


def norm_lines(path: pathlib.Path):
    """规范化内容：去行尾空白、丢掉来源戳行。"""
    text = path.read_text(encoding="utf-8", errors="replace")
    return [ln.rstrip() for ln in text.splitlines() if not STAMP.match(ln)]


def library_files(lib: pathlib.Path):
    """上游文件表：{相对上游根的路径: 绝对路径}（inc/ctl/*.hpp + src/*.cpp）。"""
    rels = sorted((lib / "inc" / "ctl").glob("*.hpp")) + sorted((lib / "src").glob("*.cpp"))
    return {p.relative_to(lib).as_posix(): p for p in rels}


def is_forwarder(path: pathlib.Path):
    """返回 (是否带标记, 是否真转发)。"""
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return (False, False)
    marked = any(FORWARDER.match(ln) for ln in lines)
    real = any(CTL_INCLUDE.match(ln) for ln in lines)
    return (marked, real)


def iter_files(root: pathlib.Path):
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if any(part in SKIP_DIRS for part in p.relative_to(root).parts):
            continue
        yield p


def detect_vendor(target: pathlib.Path, explicit: str):
    if explicit:
        v = (target / explicit).resolve()
        return v if (v / "inc" / "ctl").is_dir() else None
    for cand in VENDOR_CANDIDATES:
        v = target / cand
        if (v / "inc" / "ctl").is_dir():
            return v
    return None


def compare(target: pathlib.Path, lib: pathlib.Path, vendor_rel: str, quiet: bool = False):
    """返回 (退出码, 输出行列表)。"""
    libs = library_files(lib)
    vendor = detect_vendor(target, vendor_rel)
    out = []
    bad, missing, ok, forwarders = [], [], [], []

    if vendor is not None:
        # ── 布局①：只比 vendor 目录；下游其它位置不许再留同名文件（除非转发头）──
        names = {pathlib.Path(rel).name for rel in libs}
        for rel, src in libs.items():
            dst = vendor / rel
            if not dst.is_file():
                missing.append(rel)
            elif norm_lines(dst) == norm_lines(src) and dst.read_text(encoding="utf-8") == src.read_text(encoding="utf-8"):
                ok.append(dst.relative_to(target).as_posix())
            elif norm_lines(dst) == norm_lines(src):
                ok.append(dst.relative_to(target).as_posix() + "  （仅来源戳行不同）")
            else:
                bad.append((rel, dst.relative_to(target).as_posix()))
        for p in iter_files(target):
            if p.name not in names:
                continue
            relp = p.relative_to(target).as_posix()
            if vendor in p.parents:
                continue
            marked, real = is_forwarder(p)
            if marked and real:
                forwarders.append(relp)
            elif marked and not real:
                bad.append((p.name, relp + "  ← 标了转发头却没 #include \"ctl/…\""))
            elif norm_lines(p) == norm_lines(lib / next(r for r in libs if pathlib.Path(r).name == p.name)):
                bad.append((p.name, relp + "  ← 旧副本仍在（要么删掉，要么改成转发头）"))
            else:
                bad.append((p.name, relp + "  ← 与上游同名但内容不同，且未标 `// ctlkit-forwarder`"))
        scope_desc = f"vendor 目录 {vendor.relative_to(target).as_posix()}"
    else:
        # ── 布局②：全树按文件名比对（旧散拷贝）──
        hits = {pathlib.Path(rel).name: [] for rel in libs}
        for p in iter_files(target):
            if p.name in hits:
                hits[p.name].append(p)
        for rel, src in libs.items():
            name = pathlib.Path(rel).name
            found = hits[name]
            if not found:
                missing.append(name)
                continue
            for dst in found:
                marked, real = is_forwarder(dst)
                if marked and real:
                    forwarders.append(dst.relative_to(target).as_posix())
                elif norm_lines(dst) == norm_lines(src):
                    ok.append(dst.relative_to(target).as_posix())
                else:
                    bad.append((name, dst.relative_to(target).as_posix()))
        scope_desc = "全树（散拷贝）"

    if not quiet:
        out.append(f"上游 {lib}")
        out.append(f"下游 {target}   上游文件 {len(libs)} 个 · 比对范围：{scope_desc}\n")
        for rel in ok:
            out.append(f"  ✅ identical  {rel}")
        for rel in forwarders:
            out.append(f"  ↪  forwarder  {rel}")
        for rel, where in bad:
            out.append(f"  ❌ differs    {where}")
        for rel in missing:
            out.append(f"  ⚠️  缺失       {rel}")
        out.append("")
        if bad or missing:
            out.append(f"❌ 不通过：差异 {len(bad)} 处 · 缺失 {len(missing)} 个")
        else:
            out.append(f"✅ 全部一致（副本 {len(ok)} 处 · 转发头 {len(forwarders)} 处）")
    return (1 if (bad or missing) else 0), out


def selftest() -> int:
    """工具自测：改坏必红。用临时目录造四种下游形态。"""
    lib = pathlib.Path(__file__).resolve().parent.parent
    fails = 0

    def case(name, mutate, expect_rc, vendor_rel=""):
        nonlocal fails
        with tempfile.TemporaryDirectory() as td:
            down = pathlib.Path(td) / "down"
            vend = down / "third_party" / "ctlkit"
            (vend / "src").parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(lib / "inc", vend / "inc")
            shutil.copytree(lib / "src", vend / "src")
            legacy = down / "inc" / "foc" / "algo"
            legacy.mkdir(parents=True, exist_ok=True)
            (legacy / "pid.hpp").write_text(
                '// ctlkit-forwarder —— 转发到上游\n#pragma once\n#include "ctl/pid.hpp"\n', encoding="utf-8")
            mutate(down, vend, legacy)
            rc, lines = compare(down, lib, vendor_rel, quiet=True)
            if rc == expect_rc:
                print(f"  ✅ {name}（退出码 {rc} 符合预期）")
            else:
                print(f"  ❌ {name}：期望退出码 {expect_rc}，实际 {rc}")
                print("\n".join("      " + ln for ln in lines))
                fails += 1

    print("downstream_diff.py 自测：")
    case("① 干净的 vendor + 转发头 → 通过", lambda d, v, l: None, 0)
    case("② vendor 里改一行 → 变红",
         lambda d, v, l: (v / "src" / "pid.cpp").write_text(
             (v / "src" / "pid.cpp").read_text(encoding="utf-8").replace("integral_", "integral_ ", 1), encoding="utf-8"), 1)
    case("③ vendor 缺一个文件 → 变红",
         lambda d, v, l: (v / "src" / "ramp.cpp").unlink(), 1)
    case("④ 转发头去掉标记（旧副本嫌疑）→ 变红",
         lambda d, v, l: (l / "pid.hpp").write_text(
             '#pragma once\n#include "ctl/pid.hpp"\n', encoding="utf-8"), 1)
    case("⑤ 标了转发头却没 include ctl/ → 变红",
         lambda d, v, l: (l / "pid.hpp").write_text(
             '// ctlkit-forwarder\n#pragma once\n', encoding="utf-8"), 1)
    def scatter(down, vend, legacy):
        shutil.rmtree(vend)
        flat = down / "legacy"
        flat.mkdir()
        for src in list((lib / "inc" / "ctl").glob("*.hpp")) + list((lib / "src").glob("*.cpp")):
            shutil.copy2(src, flat / src.name)

    case("⑥ 散拷贝布局（无 vendor 目录，全树按文件名比对）→ 通过", scatter, 0)
    case("⑦ 散拷贝里改一行 → 变红",
         lambda d, v, l: (scatter(d, v, l), (d / "legacy" / "ramp.cpp").write_text(
             (d / "legacy" / "ramp.cpp").read_text(encoding="utf-8") + "// 偷偷改一行\n", encoding="utf-8")), 1)
    print("自测通过 ✅" if fails == 0 else f"自测失败 ❌（{fails} 例）")
    return 0 if fails == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="校验下游 ctlkit 副本与上游一致（vendor 目录 / 转发头 / 来源戳）")
    ap.add_argument("target", nargs="?", help="下游仓库路径（--selftest 时可省略）")
    ap.add_argument("--lib", default=str(pathlib.Path(__file__).resolve().parent.parent),
                    help="上游 ctlkit 路径（默认本脚本所在仓库）")
    ap.add_argument("--vendor", default="", help="下游里 vendor 目录的相对路径（默认自动探测 third_party/ctlkit）")
    ap.add_argument("--selftest", action="store_true", help="跑工具自测（不接触真实下游）")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    lib = pathlib.Path(args.lib).resolve()
    target = pathlib.Path(args.target or ".").resolve()
    if not (lib / "inc" / "ctl").is_dir():
        sys.exit(f"❌ 上游路径不对（找不到 inc/ctl）：{lib}")
    if not target.is_dir():
        sys.exit(f"❌ 下游路径不存在：{target}")

    rc, lines = compare(target, lib, args.vendor)
    print("\n".join(lines))
    return rc


if __name__ == "__main__":
    sys.exit(main())
