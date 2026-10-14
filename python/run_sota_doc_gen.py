"""
python/run_sota_doc_gen.py
==========================
Generates SOTA_BMS_MASTER_NOTEBOOK.docx from docs/SOTA_BMS_MASTER_NOTEBOOK.md
using relative paths (no hardcoded directories).
"""

import os
from generate_sota_docx import markdown_to_docx

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
md_src = os.path.join(repo_root, "docs", "SOTA_BMS_MASTER_NOTEBOOK.md")
target_docx = os.path.join(repo_root, "docs", "SOTA_BMS_MASTER_NOTEBOOK.docx")
title = "Adaptive EKF & Real Measured Cell Data Verification Master Notebook"

if os.path.exists(md_src):
    markdown_to_docx(md_src, target_docx, title)
    print(f"Generated docx at: {target_docx}")
else:
    print(f"Source markdown not found: {md_src}")
