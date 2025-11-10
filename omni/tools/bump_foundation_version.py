import sys
import re

if len(sys.argv) < 4:
    print("Usage: bump_foundation_version.py <path> <old_version> <new_version>")
    sys.exit(1)

file_foundation_path = f"{sys.argv[1]}/omni.physx.foundation/config/extension.toml"
old_version = sys.argv[2]
new_version = sys.argv[3]

# Bump omni.physx.foundation version
with open(file_foundation_path, "r", encoding="utf-8") as f:
    content = f.read()

# Only replace the version string on the line starting with version =
pattern = rf'^(version = "{re.escape(old_version)}")$'
replacement = rf'version = "{new_version}"'
new_content, count = re.subn(pattern, replacement, content, count=1, flags=re.MULTILINE)
if count == 0:
    print(f"Version string 'version = \"{old_version}\"' not found in file.")
    sys.exit(1)

with open(file_foundation_path, "w", encoding="utf-8") as f:
    f.write(new_content)
print(f"Bumped omni.physx.foundation version in {file_foundation_path} to {new_version}")

