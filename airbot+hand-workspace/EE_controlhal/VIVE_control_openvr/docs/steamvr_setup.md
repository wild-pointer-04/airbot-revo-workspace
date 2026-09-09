# SteamVR host setup (tracker-only, no HMD)

This package talks to **SteamVR via OpenVR**. SteamVR must already be running
on the Ubuntu host. Do **not** run SteamVR inside the AIRBOT Docker container.

Python scripts here never rewrite SteamVR config files; edit them yourself if needed.

## Launch / stop

From `VIVE_control_openvr/`:

```bash
# start (preferred)
./scripts/start_steamvr.sh

# or manually
steam -applaunch 250820

# or direct (needs Steam runtime env; prefer steam -applaunch)
~/.local/share/Steam/steamapps/common/SteamVR/bin/vrstartup.sh

# stop when finished
./scripts/stop_steamvr.sh
```

Confirm processes:

```bash
pgrep -a vrserver
pgrep -a vrmonitor
```

Then:

```bash
python3 scripts/list_trackers.py
```

## Tracker-only (no HMD): Null Driver

Config file (host):

```text
~/.local/share/Steam/config/steamvr.vrsettings
```

Useful keys under `"steamvr"` / `"driver_null"`:

| Key | Suggested |
|-----|-----------|
| `steamvr.requireHmd` | `false` |
| `steamvr.activateMultipleDrivers` | `true` |
| `driver_null.enable` | `true` |

Example fragment (merge carefully; backup the file first):

```json
{
  "steamvr": {
    "requireHmd": false,
    "activateMultipleDrivers": true
  },
  "driver_null": {
    "enable": true
  }
}
```

Restart SteamVR after editing.

## Stuck on Room Setup (tracker-only)

Null Driver often forces **SteamVR Room Setup**. This machine now has:

- `~/.local/share/Steam/config/chaperone_info.vrchap` (minimal standing play area)
- `start_steamvr.sh` auto-closes the Room Setup popup after `vrserver` is up

Manual close if it still appears:

```bash
pkill -f steamvr_room_setup
```

Useful `steamvr.vrsettings` keys already applied on this host (backup first if you edit):

| Key | Value |
|-----|--------|
| `steamvr.requireHmd` | `false` |
| `steamvr.activateMultipleDrivers` | `true` |
| `steamvr.idleTimeoutSeconds` | `0` |
| `steamvr.enableHomeApp` | `false` |
| `driver_null.enable` | `true` |

If tracking stays `OutOfRange` with green tracker LEDs, reset lighthouse DB
(backup under `~/.local/share/Steam/config/lighthouse.bak.*`) and place both
trackers so two base stations (A/B channels) can see the optical sensors.

## Dongle ownership

Only **one** consumer may open the Watchman dongles (SteamVR / this package).
Do not leave a `survive-cli` / libsurvive process running at the same time.

If `list_trackers.py` fails with `openvr.init failed`, check:

1. SteamVR is running (`./scripts/start_steamvr.sh`)
2. No leftover libsurvive / `survive-cli` process
3. Trackers powered; SteamVR Devices page shows Generic Trackers

## `list_trackers` shows (none) but USB dongles exist

SteamVR log often has:

```text
lighthouse: Attached HID Devices:
lighthouse:     No devices
Driver lighthouse has no suitable devices.
```

Cause: libsurvive previously **unbound `usbhid`** from the Watchman
interfaces (`driver: NONE`). A zombie `survive-cli` is a common leftover.
OpenVR then sees only the Null HMD.

Check:

```bash
# interfaces should list usbhid, not NONE
for i in /sys/bus/usb/devices/*:1.*/; do
  v=$(cat "${i%/}/../idVendor" 2>/dev/null) || continue
  [ "$v" = "28de" ] || continue
  echo -n "$(basename "$i") -> "
  readlink "${i}driver" 2>/dev/null || echo NONE
done
```

Fix (pick one):

1. **Unplug both Watchman dongles, wait 2s, plug back in**, then restart SteamVR.
2. Or rebind (needs sudo):

```bash
sudo pkill -9 -f survive || true
for iface in 1-1.2:1.0 1-1.2:1.1 1-1.2:1.2 1-1.3:1.0 1-1.3:1.1 1-1.3:1.2; do
  # adjust iface names from the check above
  echo "$iface" | sudo tee /sys/bus/usb/drivers/usbhid/bind
done
./scripts/stop_steamvr.sh && ./scripts/start_steamvr.sh
python3 scripts/list_trackers.py
```

Do not run a leftover libsurvive process and SteamVR against the same dongles
at the same time.
