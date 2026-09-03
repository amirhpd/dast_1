#!/bin/bash
# Step 4 -- install the VS Code extensions and drop the workspace config in.
# Safe to re-run. Requires the `code` CLI on PATH.
set -euo pipefail

cd "$(dirname "$0")"
WS="$(cd .. && pwd)"

command -v code >/dev/null || { echo "ERROR: 'code' not on PATH. Install VS Code first." >&2; exit 1; }

grep -vE '^\s*(#|$)' installed_vscode_extensions.txt | while read -r ext; do
    code --install-extension "${ext}" --force
done

# .vscode/ is gitignored, so the config templates live here and get copied in.
mkdir -p "${WS}/.vscode"
cp vscode_config/settings.json vscode_config/c_cpp_properties.json "${WS}/.vscode/"

echo
echo "Extensions installed and ${WS}/.vscode populated."
echo "IntelliSense needs the compile database -- build with:"
echo "    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
