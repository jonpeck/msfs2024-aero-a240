#!/usr/bin/env python3
"""Refresh manifest.json and layout.json for the Community package.

MSFS only loads files that are listed in layout.json (with their exact size), so run
this after ANY change inside package/local-aircraft-a240/.

Usage:  py tools/build_package.py
"""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = os.path.join(ROOT, "package", "local-aircraft-a240")
SKIP = {"layout.json", "manifest.json"}

MANIFEST = {
    "dependencies": [],
    "content_type": "AIRCRAFT",
    "title": "Aero A240",
    "manufacturer": "Aero",
    "creator": "Community",
    "package_version": "0.1.0",
    "minimum_game_version": "1.0.0",
    "release_notes": {"neutral": {"LastUpdate": "", "OlderHistory": ""}},
}


def filetime(path):
    """Windows FILETIME: 100 ns ticks since 1601-01-01."""
    return int(os.path.getmtime(path) * 10_000_000) + 116_444_736_000_000_000


def main():
    content, total = [], 0
    for folder, _, files in os.walk(PACKAGE):
        for name in files:
            full = os.path.join(folder, name)
            rel = os.path.relpath(full, PACKAGE).replace(os.sep, "/")
            if rel in SKIP:
                continue
            size = os.path.getsize(full)
            total += size
            content.append({"path": rel, "size": size, "date": filetime(full)})
    content.sort(key=lambda e: e["path"].lower())

    with open(os.path.join(PACKAGE, "layout.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump({"content": content}, f, indent=2)
    manifest = dict(MANIFEST, total_package_size=str(total).zfill(20))
    with open(os.path.join(PACKAGE, "manifest.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(manifest, f, indent=2)
    print("layout.json: %d files, %d bytes" % (len(content), total))
    for e in content:
        print("  %8d  %s" % (e["size"], e["path"]))


if __name__ == "__main__":
    main()
