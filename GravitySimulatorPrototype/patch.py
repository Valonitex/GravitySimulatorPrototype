import sys

with open('physics2.cpp', 'r') as f:
    lines = f.readlines()

with open('test_fmm2.cpp', 'r') as f:
    test_lines = f.readlines()

# Extract fmm namespace from test_fmm2.cpp
start_fmm = -1
end_fmm = -1
for i, line in enumerate(test_lines):
    if line.startswith('namespace fmm {'):
        start_fmm = i
    elif start_fmm != -1 and line.startswith('}'):
        end_fmm = i
        break

fmm_content = test_lines[start_fmm:end_fmm+1]

# Find bounds in physics2.cpp
start_phys = -1
end_phys = -1
for i, line in enumerate(lines):
    if line.startswith('namespace fmm {'):
        start_phys = i
    elif start_phys != -1 and line.startswith('} // namespace fmm'):
        end_phys = i
        break

new_lines = lines[:start_phys] + fmm_content + lines[end_phys+1:]

with open('physics2.cpp', 'w') as f:
    f.writelines(new_lines)
