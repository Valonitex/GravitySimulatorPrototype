with open('physics2.cpp', 'r') as f:
    lines = f.readlines()

start = -1
for i, line in enumerate(lines):
    if line.startswith('namespace fmm {'):
        start = i
        break

if start != -1:
    end = -1
    braces = 0
    for i in range(start, len(lines)):
        braces += lines[i].count('{')
        braces -= lines[i].count('}')
        if braces == 0:
            end = i
            break
    if end != -1:
        lines[end] = '} // namespace fmm\n'
        with open('physics2.cpp', 'w') as f:
            f.writelines(lines)
