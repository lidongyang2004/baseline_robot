#!/usr/bin/env python3
"""Save an unmodified X11 application window, recording its capture provenance."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

from PyQt5.QtWidgets import QApplication


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--window", type=lambda value: int(value, 0), required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--description", required=True)
args = parser.parse_args()
if args.output.exists():
    parser.error("Output exists; preserve previous screenshots by choosing a new name")
application = QApplication([])
pixmap = application.primaryScreen().grabWindow(args.window)
if pixmap.isNull():
    parser.error("The requested application window could not be captured")
args.output.parent.mkdir(parents=True, exist_ok=True)
if not pixmap.save(str(args.output), "PNG"):
    parser.error("PNG save failed")
record = {
    "captured_at_utc": datetime.now(timezone.utc).isoformat(),
    "display": os.environ.get("DISPLAY"),
    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
    "window_id": hex(args.window),
    "window_title": subprocess.check_output(
        ["xprop", "-id", hex(args.window), "WM_NAME"], text=True).strip(),
    "output": str(args.output),
    "size_px": [pixmap.width(), pixmap.height()],
    "description": args.description,
    "capture_method": "QScreen.grabWindow; application client pixels, no postprocessing",
    "sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
}
with (Path(__file__).resolve().parent / "screenshots.jsonl").open("a") as stream:
    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
print(json.dumps(record, ensure_ascii=False, indent=2))
