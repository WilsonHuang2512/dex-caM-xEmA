import re

input_file = "xcamera.h"
output_file = "xcamera_neutral.h"

with open(input_file, "r", encoding="utf-8") as f:
    lines = f.readlines()

# ---------- 处理NEUTRAL_MODE ----------
output = []
keep_stack = []
neutral_stack = []

for line in lines:
    stripped = line.strip()

    if stripped.startswith("#ifdef NEUTRAL_MODE"):
        neutral_stack.append(True)
        keep_stack.append(True)
        output.append("\n")
        continue

    elif stripped.startswith("#else") and neutral_stack:
        # 切换为删除状态
        keep_stack[-1] = False
        output.append("\n")
        continue

    elif stripped.startswith("#endif") and neutral_stack:
        neutral_stack.pop()
        keep_stack.pop()
        output.append("\n")
        continue

    if keep_stack and not keep_stack[-1]:
        output.append("\n")
    else:
        output.append(line)

# ---------- 处理LUMOS_API ----------
first_lumos_index = None
for i, line in enumerate(output):
    if "XEMA_API" in line:
        first_lumos_index = i
        break

if first_lumos_index is not None:
    start = max(first_lumos_index - 3, 0)
    end = min(first_lumos_index + 5, len(output))  # 前2 + 当前 + 后4
    for i in range(start, end):
        output[i] = "\n"

# ---------- 删除 #define LumosCameraSelect CameraSelect ----------
for i, line in enumerate(output):
    if "#define XemaEngine Engine" in line:
        output[i] = "\n"
for i, line in enumerate(output):
    if "#define XemaColor Color" in line:
        output[i] = "\n"

# ---------- 替换 ----------
for i, line in enumerate(output):
    line = line.replace("__XEMA_LCAMERA_H__", "__CAMERA_LCAMERA_H__")
    line = line.replace("XemaEngine", "Engine")
    line = line.replace("XemaColor", "Color")
    output[i] = line

# ---------- CAMERA_API 宏块加上 #endif，缩进与 #ifdef 一致 ----------
i = 0
while i < len(output):
    line = output[i]
    stripped_line = line.lstrip()
    indent = line[:len(line) - len(stripped_line)]
    if stripped_line.startswith("#ifdef _WIN32") and i + 1 < len(output):
        j = i + 1
        while j < len(output):
            if "#elif __linux" in output[j]:
                k = j + 1
                while k < len(output) and output[k].lstrip().startswith("#define CAMERA_API"):
                    k += 1
                output.insert(k, f"{indent}#endif\n")
                break
            j += 1
        i = k
    i += 1

# ---------- 写入文件 ----------
with open(output_file, "w", encoding="utf-8") as f:
    f.writelines(output)

print(f"输出文件已生成: {output_file}")
