from __future__ import annotations

import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

PACK_ID = "Keil.STM32F4xx_DFP.2.13.0"


def fix_project(project: Path) -> bool:
    text = project.read_text(encoding="utf-8-sig")
    original = text
    has_rte = re.search(r"(?s)\s*<RTE>.*?</RTE>", text) is not None

    if has_rte:
        text = re.sub(r"(?s)\s*<RTE>.*?</RTE>", "", text)

    pack_tag = f"<PackID>{PACK_ID}</PackID>"
    if re.search(r"<PackID>.*?</PackID>", text):
        text = re.sub(r"<PackID>.*?</PackID>", pack_tag, text)
    else:
        text = re.sub(r"(<Device>[^<]+</Device>)", r"\1" + pack_tag, text, count=1)

    if text == original:
        print(f"[post-generate] already fixed: {project}")
        return False

    backup = project.with_name(
        f"{project.name}.bak_{datetime.now():%Y%m%d_%H%M%S}"
    )
    shutil.copy2(project, backup)
    project.write_text(text, encoding="utf-8", newline="")
    print(f"[post-generate] fixed: {project}")
    print(f"[post-generate] backup: {backup}")
    return True


def main() -> int:
    root = Path(__file__).resolve().parent
    mdk_dir = root / "MDK-ARM"
    if not mdk_dir.is_dir():
        print(f"[post-generate] MDK-ARM not found: {mdk_dir}")
        return 0

    projects = sorted(mdk_dir.glob("*.uvprojx"))
    if not projects:
        print(f"[post-generate] no .uvprojx in: {mdk_dir}")
        return 0

    for project in projects:
        try:
            fix_project(project)
        except Exception as exc:  # keep CubeMX generation from failing
            print(f"[post-generate] ERROR {project}: {exc}")

    return 0


if __name__ == "__main__":
    sys.exit(main())