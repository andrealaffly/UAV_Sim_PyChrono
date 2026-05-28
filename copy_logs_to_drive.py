"""
===============================================================
🛠 Log Copier Script - copy_logs_to_drive.py
===============================================================
This script copies log files from a source folder to a destination folder
based on a specific date (defaults to today), **without overwriting** files
that already exist at the destination.
---------------------------------------------------------------
📦 USAGE
---------------------------------------------------------------
Run from the command line or via a .bat file.

Default (today's date, using private repo):
  python copy_logs_to_drive.py

Copy yesterday's logs (using private repo):
  python copy_logs_to_drive.py --yesterday

Copy logs for a specific date (format: YYYYMMDD):
  python copy_logs_to_drive.py --date 20250611

Use logs from the public repo instead of the private one:
  python copy_logs_to_drive.py --public-repo

Combine options:
  python copy_logs_to_drive.py --date 20250611 --public-repo
  python copy_logs_to_drive.py --yesterday --public-repo
---------------------------------------------------------------
🧩 ARGUMENTS
---------------------------------------------------------------
--date YYYYMMDD   Optional
  Use this to copy logs from a specific date.
  Example: --date 20250611

--yesterday       Optional
  Use this flag to copy logs from yesterday.
  Note: If both --date and --yesterday are provided, --yesterday takes priority.

--public-repo     Optional
  Use the public repo as the source directory.
  If not set, the default is the private repo.
---------------------------------------------------------------
📂 SOURCE / DESTINATION STRUCTURE
---------------------------------------------------------------
Paths are built using:
  source_base / <year> / <month> / <day_folder>
  destination_base / <year> / <month> / <day_folder>

By default:
  source_base = private repo path
  If --public-repo is set, source_base = public repo path

You can modify `source_base` and `destination_base` to match your setup.
---------------------------------------------------------------
📝 NOTES
---------------------------------------------------------------
- Files that already exist at the destination will be skipped.
- The script creates subfolders as needed to mirror the source structure.
- You can automate this with Windows Task Scheduler or wrap it in a .bat file.
===============================================================
"""

import os
import shutil
from datetime import datetime, timedelta
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Copy logs without overwriting existing files.")
parser.add_argument("--date", type=str, help="Date in YYYYMMDD format (default: today)")
parser.add_argument("--yesterday", action="store_true", help="Use yesterday's date")
parser.add_argument("--public-repo", action="store_true", help="Use the public repo path as the source")
args = parser.parse_args()

# Determine the date to use
if args.yesterday:
  date = datetime.today() - timedelta(days=1)
elif args.date:
  try:
    date = datetime.strptime(args.date, "%Y%m%d")
  except ValueError:
    raise ValueError("Invalid date format. Use YYYYMMDD (e.g., 20250611).")
else:
  date = datetime.today()

# Extract year, month, and day_folder
year = date.strftime("%Y")
month = date.strftime("%m")
day_folder = date.strftime("%Y%m%d")

# Define source and destination paths
if args.public_repo:
  source_base = r"/mnt/d/Virginia-Tech-PhD/PHD_research/PyChrono/UAV_Sim_PyChrono/logs"
else:
  source_base = r"/mnt/d/Virginia-Tech-PhD/PHD_research/PyChrono/UAV_Sim_PyChrono/logs"

destination_base = r"/mnt/d/Virginia-Tech-PhD/PHD_research/PyChrono/ACSL-flightstack-accessories/pychrono"

source = os.path.join(source_base, year, month, day_folder)
destination = os.path.join(destination_base, year, month, day_folder)

print(f"Copying from:\n  {source}\nto:\n  {destination}\n")

# Copy files that do not already exist
for root, dirs, files in os.walk(source):
  for file in files:
    src_path = os.path.join(root, file)
    rel_path = os.path.relpath(src_path, source)
    dst_path = os.path.join(destination, rel_path)

    if not os.path.exists(dst_path):
      os.makedirs(os.path.dirname(dst_path), exist_ok=True)
      shutil.copy2(src_path, dst_path)
      print(f"✅ Copied: {rel_path}")
    else:
      print(f"⏩ Skipped (already exists): {rel_path}")

