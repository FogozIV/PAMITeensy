import re
import os

def after_build(source, target, env):
    map_file = os.path.join(env.subst(env['BUILD_DIR']), "firmware.map")
    print(f"Mapping file: {map_file}")
    if not os.path.isfile(map_file):
        print("Map file not found.")
        return

    print("\n[INFO] Top RAM1 (0x2000xxxx) Usage:")
    entries = []
    with open(map_file) as f:
        for line in f:
            match = re.search(r"0x2000[0-9a-fA-F]+ +0x([0-9a-fA-F]+) +(\S+)", line)
            if match:
                size = int(match.group(1), 16)
                symbol = match.group(2)
                entries.append((size, symbol))

    entries.sort(reverse=True)
    for size, symbol in entries[:20]:
        print(f"{symbol:<40} {size:>10} bytes")

# Register the hook
Import("env")
env.AddPostAction("buildprog", after_build)