# -*- coding: utf-8 -*-
import argparse
import os
import glob

import re

def simplify_line(line: str) -> str:
    """
    精简行内容：
    1. 保留前12位编码。
    2. 删除 R1 决策（从 '0' 开始及其后的内容）。
    3. 删除拾取决策（括号及其内部内容）。
       - 伪台阶拾取如 '(3)' -> 删除
       - 真台阶拾取如 '5(8)' -> '5'
       - 多重拾取如 '5(4 8)' -> '5'
    """
    parts = line.strip().split()
    if not parts:
        return ""

    code = parts[0]
    rest = parts[1:]

    # 1. 截断 '0' 及其之后的部分 (R1 决策)
    try:
        r1_idx = rest.index('0')
        rest = rest[:r1_idx]
    except ValueError:
        pass
        
    # 重新组合成字符串以便正则处理
    path_str = " ".join(rest)
    
    # 2. 正则替换：去除 (...) 及其内容
    # \(.*?\) 匹配括号及内部非贪婪
    path_str = re.sub(r'\(.*?\)', '', path_str)
    
    # 3. 再次 split 并过滤空字符串，确保格式整洁
    final_tokens = path_str.split()
    
    if final_tokens:
        return f"{code} {' '.join(final_tokens)}"
    else:
        return code

def process_file(filepath: str, enable_sort: bool, enable_simplify: bool) -> None:
    if not os.path.exists(filepath):
        print(f"文件不存在: {filepath}")
        return

    print(f"正在处理: {filepath} ...")

    # 读取原文件（尽量用 utf-8，失败再退回 gbk）
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
    except UnicodeDecodeError:
        with open(filepath, "r", encoding="gbk") as f:
            lines = [line.strip() for line in f if line.strip()]

    # 1. 排序
    if enable_sort:
        # 根据每行第一个部分（编码）进行排序
        # 编码是长度固定 12 的数字字符串，按字符串排序即可
        lines.sort(key=lambda x: x.split()[0])

    # 2. 精简
    if enable_simplify:
        lines = [simplify_line(line) for line in lines]

    # 3. 生成输出文件名（不覆盖原文件）
    dir_name, file_name = os.path.split(filepath)
    name_root, ext = os.path.splitext(file_name)

    suffix = ""
    if enable_simplify:
        suffix += "_simplied"   # 按你要求的拼写
    if enable_sort:
        suffix += "_sorted"

    new_file_name = f"{name_root}{suffix}{ext}"
    output_path = os.path.join(dir_name, new_file_name)

    with open(output_path, "w", encoding="utf-8") as f:
        for line in lines:
            f.write(line + "\n")

    print(f"已生成: {output_path}")

def main() -> None:
    parser = argparse.ArgumentParser(description="处理 placements txt 文件：排序与精简。")
    parser.add_argument(
        "files",
        nargs="*",
        help="要处理的文件路径。如果不指定，将搜索当前目录下所有 placements_and_paths_*.txt",
    )
    parser.add_argument("--sort", action="store_true", help="启用排序：根据地图编码大小排序")
    parser.add_argument("--simplify", action="store_true", help="启用精简：删除拾取决策和 R1 决策")

    args = parser.parse_args()

    target_files = args.files

    # 如果未指定文件，自动搜索
    if not target_files:
        target_files = glob.glob("placements_and_paths_*.txt")
        if not target_files:
            print("未指定文件，且当前目录下未找到 placements_and_paths_*.txt 文件。")
            return

    if not args.sort and not args.simplify:
        print("警告：未启用排序 (--sort) 也未启用精简 (--simplify)，不会执行任何操作。")
        print("请至少指定一个操作。")
        return

    for fpath in target_files:
        process_file(fpath, args.sort, args.simplify)

if __name__ == "__main__":
    main()