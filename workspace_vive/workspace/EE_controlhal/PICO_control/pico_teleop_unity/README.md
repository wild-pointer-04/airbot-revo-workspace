# PicoTeleop Unity

PICO 4 / PICO 4 Ultra Enterprise controller poses → UDP JSON → Ubuntu `middle_interface`.

This project **does not** talk to a robot SDK. Coordinates stay in XR tracking space. Quaternion order is `[x, y, z, w]`.

## SDK

| Item | Value |
| --- | --- |
| Unity | 2022.3 LTS (Integration SDK minimum is 2021.3.26f1) |
| PICO SDK | `PICO-Unity-Integration-SDK-main` (`com.unity.xr.picoxr` 3.4.0) |
| OpenXR | Unity OpenXR Plugin (`com.unity.xr.openxr`) |
| Build target | Android, IL2CPP, ARM64, min API 29 |

Do **not** import `PICO-Unity-OpenXR-SDK-main`. Integration SDK already provides PICO OpenXR features when the Unity OpenXR loader is enabled (`PICO_OPENXR_SDK`).

Local package path in `Packages/manifest.json`:

```text
com.unity.xr.picoxr: file:../PICO-Unity-Integration-SDK-main
```

## Data flow

```text
PICO HMD / controllers
  → Unity XR InputDevices (OpenXR)
  → PicoTeleopPacket (JSON)
  → UDP  (default 60 Hz, port 15000)
  → middle_interface
```

## Open the project

Unity Hub is installed. On this Ubuntu 24.04 start it with:

```bash
/home/dt/ego_ARX/EE_controlhal/PICO_control/pico_teleop_unity/open_unity_hub.sh
```

Install **2022.3 LTS** plus **Android Build Support**. Then Hub → **Open** → `pico_teleop_unity`.

Scene: `Assets/PicoTeleop/Scenes/PicoTeleop.unity`

1. Build Settings → Android → Switch Platform
2. Player: IL2CPP, ARM64, Minimum API 29
3. XR Plug-in Management → Android → **OpenXR** (not the PICO XR loader)
4. OpenXR: enable **PICO XR**, plus **PICO4** / **PICO4 Ultra** controller profiles

## Ubuntu IP / port / rate

USB (this machine): run `bash usb_adb_reverse.sh`, then:

```text
Receiver Ip    = 127.0.0.1
Receiver Port  = 15000
Send Rate Hz   = 60
```

WiFi instead: set Receiver Ip to the Ubuntu LAN address (currently `192.168.5.23`).

Select GameObject `PicoTeleopManager` to edit these fields.

## Confirm tracking

Wear the headset. The world-space debug panel shows:

```text
Target / Send Hz / Seq
Right Tracking: OK | LOST
Left Tracking:  OK | LOST
Right / Left XYZ
```

Seq must increase. Right/Left XYZ must change when you move the controllers.

## Build APK

File → Build Settings → Android → Build. Install on PICO 4 / PICO 4 Ultra Enterprise. PICO and Ubuntu must be on the same LAN.

## Layout

```text
Assets/PicoTeleop/
  Scripts/Input|Protocol|Network|Core|Debug
  Scenes/PicoTeleop.unity
  Editor/
```

Namespace: `EgoARX.PicoTeleop.*`
