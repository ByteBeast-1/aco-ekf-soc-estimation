import os
import shutil
from generate_sota_docx import markdown_to_docx

md_src = r"c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation\SOTA_BMS_MASTER_NOTEBOOK.md"

dest_dirs = [
    r"c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation_transfer\aco-ekf-soc-estimation",
    r"c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation_transfer\aco-ekf-soc-estimation\docs",
    r"c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation",
    r"c:\Users\sande\OneDrive\Desktop\aco-ekf-soc-estimation\docs"
]

title = "SOTA Adaptive EKF & NASA Hardware Verification Master Notebook"

for d in dest_dirs:
    os.makedirs(d, exist_ok=True)
    target_md = os.path.join(d, "SOTA_BMS_MASTER_NOTEBOOK.md")
    if target_md != md_src:
        shutil.copy2(md_src, target_md)
        print(f"Copied MD to: {target_md}")
    target_docx = os.path.join(d, "SOTA_BMS_MASTER_NOTEBOOK.docx")
    markdown_to_docx(target_md, target_docx, title)

print("All SOTA BMS documents generated successfully!")
