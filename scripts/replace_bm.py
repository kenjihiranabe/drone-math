#!/usr/bin/env python3
from pathlib import Path


def replace_bm(text: str) -> str:
    return text.replace(r"\\bm{", r"\\boldsymbol{")


def main() -> int:
    changed = 0
    for path in Path('.').rglob('*.md'):
        if '.git' in path.parts:
            continue
        original = path.read_text(encoding='utf-8')
        updated = replace_bm(original)
        if updated != original:
            path.write_text(updated, encoding='utf-8')
            changed += 1
            print(f"updated: {path}")
    print(f"done: changed {changed} markdown file(s)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
