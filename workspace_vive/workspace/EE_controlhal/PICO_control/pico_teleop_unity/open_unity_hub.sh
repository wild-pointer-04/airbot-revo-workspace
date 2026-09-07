#!/usr/bin/env bash
# Open Unity Hub on dt2's own screen. Do not use Cursor's BROWSER helper.
set -euo pipefail
unset BROWSER
unset VSCODE_IPC_HOOK_CLI
export DISPLAY="${DISPLAY:-:1}"
if [[ -x /home/dt/.local/firefox/firefox ]]; then
  export BROWSER=/home/dt/.local/firefox/firefox
elif command -v epiphany >/dev/null; then
  export BROWSER=epiphany
fi
exec /usr/lib/unityhub/unityhub --no-sandbox "$@"
