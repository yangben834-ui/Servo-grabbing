#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 poses.txt 里每一行的 rx、ry 取反（x/y/z 和 rz 不动）。

用法:
    python3 negate_rx_ry.py H3001/poses.txt
    python3 negate_rx_ry.py H3001/poses.txt -o /tmp/poses_neg.txt

输入行格式为逗号分隔的 6 列 [x, y, z, rx, ry, rz]。
只重写第 4、5 列，其余字符原样保留 —— 包括逗号周围的对齐空格/制表符，
以及数字原有的全部有效位（走字符串改符号，不经过 float 重新格式化）。

默认另存为 <输入文件名>_rxry_neg.txt，不覆盖原文件。

注意: 这个操作等价于对旋转矩阵做共轭 R -> Rz(pi)·R·Rz(pi)^T。
它会实实在在改变标定结果（H3001 实测：解出的相机位置偏移 1439 mm、
旋转翻转 179.6°），但所有基于相对旋转角的自洽检查都看不出异常 ——
因为共轭保旋转角。
"""

import argparse
import os
import re
import sys

# 一列里允许前后有空白，数值部分只做符号翻转
_FIELD_RE = re.compile(r"^(\s*)([-+]?[0-9][0-9eE+.\-]*)(\s*)$")


def negate_rx_ry(pose):
    """把 [x, y, z, rx, ry, rz] 里的 rx、ry 取反，返回新列表。

    需要直接对列表用（而不是读写文件）时用这个函数。
    """
    values = list(pose)
    if len(values) != 6:
        raise ValueError(f"位姿应为 6 个元素，实际得到 {len(values)} 个。")
    values[3] = -values[3]
    values[4] = -values[4]
    return values


def _flip_sign(text):
    """翻转一个数字字符串的符号，保留原有数字字符与前后空白。

    走字符串而不是 float，避免重新格式化丢掉末尾有效位。
    """
    match = _FIELD_RE.match(text)
    if match is None:
        raise ValueError(f"无法解析为数值: {text!r}")
    lead, number, tail = match.groups()
    if float(number) == 0.0:          # 0 取反还是 0，别产出 "-0.0"
        return text
    if number[0] == "-":
        return f"{lead}{number[1:]}{tail}"
    if number[0] == "+":
        return f"{lead}-{number[1:]}{tail}"
    return f"{lead}-{number}{tail}"


def negate_line(line):
    """处理 poses.txt 的一行，只改第 4、5 列。空行原样返回。"""
    body = line.rstrip("\n")
    if not body.strip():
        return body
    fields = body.split(",")
    if len(fields) != 6:
        raise ValueError(f"应有 6 列，实际 {len(fields)} 列: {body!r}")
    fields[3] = _flip_sign(fields[3])
    fields[4] = _flip_sign(fields[4])
    return ",".join(fields)


def default_output_path(pose_file):
    root, ext = os.path.splitext(pose_file)
    return f"{root}_rxry_neg{ext or '.txt'}"


def main():
    parser = argparse.ArgumentParser(
        description="把 poses.txt 里每行的 rx、ry 取反（rz 和 x/y/z 不动）。"
    )
    parser.add_argument("pose_file", help="输入文件，逗号分隔 6 列：x,y,z,rx,ry,rz")
    parser.add_argument("-o", "--out", default=None,
                        help="输出文件；默认 <输入名>_rxry_neg.txt")
    parser.add_argument("--in-place", action="store_true",
                        help="直接覆盖输入文件（默认另存，不覆盖）")
    args = parser.parse_args()

    if not os.path.isfile(args.pose_file):
        print(f"找不到文件: {args.pose_file}", file=sys.stderr)
        return 1
    if args.in_place and args.out:
        print("--in-place 与 -o 不能同时使用。", file=sys.stderr)
        return 1

    out_path = args.pose_file if args.in_place else (args.out or default_output_path(args.pose_file))

    with open(args.pose_file, encoding="utf-8") as f:
        lines = f.readlines()

    results, changed = [], 0
    for lineno, line in enumerate(lines, start=1):
        try:
            new_line = negate_line(line)
        except ValueError as exc:
            print(f"第 {lineno} 行处理失败: {exc}", file=sys.stderr)
            return 1
        if new_line != line.rstrip("\n"):
            changed += 1
        results.append(new_line + "\n")

    with open(out_path, "w", encoding="utf-8") as f:
        f.writelines(results)

    print(f"已写入: {out_path}")
    print(f"共 {len([l for l in lines if l.strip()])} 行位姿，改动 {changed} 行")
    if lines:
        print("\n前两行对照:")
        for line in lines[:2]:
            if not line.strip():
                continue
            print(f"  原: {line.rstrip()}")
            print(f"  新: {negate_line(line)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
