#!/usr/bin/env bash
# Finish Unity Hub login on THIS SSH machine after the browser ran on your laptop.
# Usage: bash complete_unity_login.sh 'unityhub://login?code=....'
set -euo pipefail
url="${1:-}"
if [[ -z "$url" ]]; then
  echo "Paste the unityhub:// URL from the laptop browser:"
  echo "  $0 'unityhub://login?code=...'"
  exit 1
fi
unset BROWSER
export DISPLAY="${DISPLAY:-:1}"
exec /usr/lib/unityhub/unityhub --no-sandbox "$url"
