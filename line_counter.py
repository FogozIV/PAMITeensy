import os

EXCLUDE_DIRS = {".pio", "libdeps", ".git", "build", ".vscode", ".idea"}
VALID_EXTENSIONS = {".cpp", ".h", ".c", ".hpp"}

def count_lines(root="."):
    total_lines = 0
    total_files = 0

    for dirpath, dirnames, filenames in os.walk(root):
        # Skip excluded directories
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]

        for filename in filenames:
            ext = os.path.splitext(filename)[1]
            if ext in VALID_EXTENSIONS:
                filepath = os.path.join(dirpath, filename)
                try:
                    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
                        lines = sum(1 for _ in f)
                        total_lines += lines
                        total_files += 1
                        print(f"{filepath}: {lines} lines")
                except Exception as e:
                    print(f"Error reading {filepath}: {e}")

    return total_files, total_lines

if __name__ == "__main__":
    files, lines = count_lines()
    print(f"\nTotal files: {files}")
    print(f"Total lines of code: {lines}")