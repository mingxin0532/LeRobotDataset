#!/usr/bin/env python3
import os
from pathlib import Path

# 指向你刚才保存成功的路径
DATASET_ROOT = "./my_dataset_final_success"

def print_tree(dir_path):
    root_path = Path(dir_path)
    if not root_path.exists():
        print(f"❌ 错误：找不到目录 {dir_path}")
        return

    print(f"📦 数据集根目录: {root_path.absolute()}\n")

    # 遍历所有文件
    for root, dirs, files in os.walk(root_path):
        level = root.replace(str(root_path), '').count(os.sep)
        indent = '│   ' * level
        folder_name = os.path.basename(root)
        
        # 打印文件夹名
        if level == 0:
            print(f"📂 {folder_name}/")
        else:
            print(f"{indent}├── 📂 {folder_name}/")
        
        # 打印文件名
        sub_indent = '│   ' * (level + 1)
        for i, f in enumerate(files):
            file_path = os.path.join(root, f)
            file_size = os.path.getsize(file_path)
            
            # 格式化文件大小
            if file_size < 1024:
                size_str = f"{file_size} B"
            elif file_size < 1024 * 1024:
                size_str = f"{file_size/1024:.1f} KB"
            else:
                size_str = f"{file_size/(1024*1024):.1f} MB"

            # 树状图连接符
            connector = "└──" if i == len(files) - 1 else "├──"
            print(f"{sub_indent}{connector} 📄 {f}  [大小: {size_str}]")

if __name__ == "__main__":
    print_tree(DATASET_ROOT)
