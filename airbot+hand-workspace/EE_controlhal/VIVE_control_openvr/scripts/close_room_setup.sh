#!/usr/bin/env bash
# Close the SteamVR Room Setup popup without touching vrserver.
set -euo pipefail
killed=0
while read -r pid; do
  [[ -n "${pid}" ]] || continue
  kill "${pid}" >/dev/null 2>&1 || true
  echo "Closed Room Setup pid=${pid}"
  killed=1
done < <(
  ps -eo pid=,args= | awk '
    /tools\/steamvr_room_setup\/linux64\/steamvr_room_setup -popupwindow/ {print $1}
  '
)
if [[ "${killed}" -eq 0 ]]; then
  echo "No Room Setup popup found."
fi
