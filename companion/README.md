# MOLE: MAVLink companion computer

Raspberry Pi 3B+ (`MOLE`, Debian 13 trixie, 64-bit), running as a MAVLink companion
computer for an ArduPilot flight controller (TBS Lucid H7).

```
FC (SERIAL7, 921600) ──UART──> /dev/serial0 ──> mavlink-router ──> TCP :5760
                                                                    ├─ check_link.py / motor_test.py
                                                                    └─ laptop GCS
```

## Quick setup (fresh Pi)

Everything in "Changes made to this Pi" below is automated by `setup.sh`. It is safe to re-run.

```bash
sudo apt-get install -y git
git clone https://github.com/JayChintala/Multirotor-Operated-Lab-Experiments-MOLE.git ~/mole
bash ~/mole/companion/setup.sh      # run as your normal user; uses sudo
sudo reboot                         # only if the script says so
bash ~/mole/companion/setup.sh      # re-run: every check should say OK
```

The script does not touch SSH, Wi-Fi or user accounts (apart from adding you to `dialout`), and never reboots on its own.

## Wiring

Tested 2026-10-04: two-way MAVLink between the Pi and the FC is working, and the motors spin with `motor_test.py`.

| Pi pin          | Lucid H7 |
|-----------------|----------|
| 8  (GPIO14 TXD) | RX7      |
| 10 (GPIO15 RXD) | TX7      |
| 6  (GND)        | GND      |

## Flight controller settings

TBS Lucid H7 running ArduCopter 4.7.2, with a TBS Lucid 6S 4-in-1 ESC.

| Parameter | Value | Why |
|---|---|---|
| `SERIAL7_PROTOCOL` | 2 | MAVLink2 on the UART wired to the Pi |
| `SERIAL7_BAUD` | 921 | 921600 baud, matching mavlink-router |
| `MOT_PWM_TYPE` | 5 | DShot300 (was 6, DShot600, when first set up). **DShot is required for the Lucid 4-in-1 ESC**: with the default 0 (PWM) the FC accepts motor tests but the motors don't spin |
| `FRAME_CLASS` / `FRAME_TYPE` | 1 / 1 | Quad X |
| `SERVO1_FUNCTION`–`SERVO4_FUNCTION` | 33–36 | Outputs 1–4 drive motors 1–4 |
| `BRD_SAFETY_DEFLT` | 0 | No safety switch |
| `SERVO_DSHOT_ESC` | 1 | Lets the FC send DShot commands to the ESC (BLHeli32/AM32 type). Without it, `SERVO_BLH_RVMASK` has no effect |
| `SERVO_BLH_MASK` | 15 | Outputs 1–4 are DShot ESCs managed by the FC |
| `SERVO_BLH_RVMASK` | 9 | Reverses outputs 1 and 4 (motors 1 and 4), so the motors spin the Quad X directions: front-right and rear-left CCW, front-left and rear-right CW (seen from above). Verified 2026-10-26 with `motor_test.py` |
| `FS_GCS_ENABLE` | 5 | Land if the GCS link is lost. With no RC receiver, the Wi-Fi link to QGC is the only control link |
| `FS_GCS_TIMEOUT` | 5 | Seconds without a GCS heartbeat before the failsafe triggers |
| `FS_OPTIONS` | 0 | GCS failsafe applies in every mode. The earlier value, 18, kept flying in Stabilize/AltHold/Loiter and in Auto missions after link loss |

Reboot the FC after changing `SERIAL7_*` or `MOT_PWM_TYPE`. With DShot working, the ESC plays its start-up tones followed by confirmation beeps once the FC has booted.

Bench notes:
- The motors spin on a 12 V bench supply (10 A limit). Expect lower top speed than on 6S.
- No RC receiver is connected yet, so the FC reports 0 RC channels.
- `BATT_LOW_VOLT` is 10.5 V (a 3S value) and the FC reports the battery monitor as unhealthy. Set the battery parameters for 6S before flying.

## Changes made to this Pi

### Packages
- `apt update` and `full-upgrade` were run (already up to date on 2026-10-04).
- Installed: git, python3-venv, python3-pip, tmux, meson, ninja-build, pkg-config, gcc, g++.

### UART
- `raspi-config nonint do_serial_cons 1` turned off the serial login console.
  `/boot/firmware/cmdline.txt` has no `console=serial0`.
- `raspi-config nonint do_serial_hw 0` turned on the UART hardware, which adds `enable_uart=1` to `/boot/firmware/config.txt`.
- `/boot/firmware/config.txt` has `dtoverlay=disable-bt` under `[all]`. This gives the full PL011 UART (`ttyAMA0`) to GPIO 14/15, so `/dev/serial0 -> ttyAMA0`.
- `hciuart.service` doesn't exist on trixie, so there was nothing to disable. `bluetooth.service` is still enabled, but has no hardware to use because of the overlay.
- `serial-getty@ttyAMA0` is disabled and inactive.
- User `molepi` is in the `dialout` group.
- Verified after a reboot: `/dev/serial0 -> ttyAMA0`, and no getty is running on it.

### Python
- Venv at `~/mav` (Python 3.13) with `pymavlink` 2.4.50 and `MAVProxy` 1.8.75.
- `~/.bashrc` ends with `source ~/mav/bin/activate`, so the venv is active in every login shell.

### mavlink-router
- Source: `~/src/mavlink-router` (with submodules), commit `2362c62`, version `v4-16-g2362c62`.
- Built with `meson setup build . --buildtype=release && ninja -C build -j2`, then `sudo ninja -C build install`.
- Installed files:
  - `/usr/bin/mavlink-routerd`
  - `/usr/lib/systemd/system/mavlink-router.service`
- Config file: `/etc/mavlink-router/main.conf`
  - UART endpoint `fc`: `/dev/serial0` at 921600
  - TCP server on port 5760, all interfaces
- The service is enabled, so it starts at boot. It runs and listens on 5760 even with no FC attached.

### Scripts
- `~/mole/companion/setup.sh`: reproduces this whole setup on a fresh Pi (see Quick setup).
- `~/mole/companion/check_link.py`: a read-only link check (see below).
- `~/mole/companion/motor_test.py`: bench motor test, one motor at a time (see below).
- `~/mole/companion/README.md`: this file.
- `~/mole/companion/mavlink-router/main.conf`: copy of `/etc/mavlink-router/main.conf` for reference. After editing, install it with `sudo install -m 644 companion/mavlink-router/main.conf /etc/mavlink-router/` and restart the service.

## Controlling mavlink-router

```bash
sudo systemctl status mavlink-router        # is it running?
sudo systemctl stop mavlink-router          # stop it (frees /dev/serial0)
sudo systemctl start mavlink-router         # start it
sudo systemctl restart mavlink-router       # restart, e.g. after editing main.conf
sudo systemctl disable mavlink-router       # don't start at boot
journalctl -u mavlink-router -f             # follow the logs
ss -ltn | grep 5760                         # confirm it's listening
```

To rebuild after an update:

```bash
cd ~/src/mavlink-router && git pull && git submodule update --init --recursive
ninja -C build -j2 && sudo ninja -C build install && sudo systemctl restart mavlink-router
```

## Checking the link

```bash
~/mav/bin/python ~/mole/companion/check_link.py     # or just: python ~/mole/companion/check_link.py
```

The script connects to `tcp:127.0.0.1:5760` and waits up to 10 s for an autopilot heartbeat. It then prints:
- vehicle type
- firmware version
- flight mode
- armed state
- GPS fix and satellite count
- battery voltage
- EKF status flags

If no heartbeat arrives, it prints a wiring/config checklist and exits with code 1. It only requests telemetry. It never arms the vehicle or sends motor commands.

Both scripts send a heartbeat to the FC before their first command. Without one, the FC ignored the first command after each new connection, and firmware, GPS and battery showed `n/a`.

## Motor test (bench)

**Remove the propellers first.** The FC must be disarmed and the ESC powered.

```bash
python ~/mole/companion/motor_test.py               # motors 1-4 in turn, 7 % for 2 s each
python ~/mole/companion/motor_test.py -m 4 -t 10    # only motor 4 (D) at 10 %
```

- Motor numbers follow ArduPilot's test order: 1=A, 2=B, 3=C, 4=D, clockwise from the front-right motor on a quad X. Use this to check each motor's position and spin direction.
- Uses `MAV_CMD_DO_MOTOR_TEST`, the same command as Mission Planner's Motor Test page. The FC stops each motor by itself after the duration.
- Safety limits: asks you to type YES; throttle capped at 15 % and duration at 5 s; refuses if the vehicle is armed; ignores GCS heartbeats; never arms. Ctrl-C stops the motor that is running.
- Before any motor command, it checks the FC answers a harmless request. If not, it stops without sending one.
- If the FC refuses, the script prints the reason (e.g. RC not calibrated, vehicle not landed).
- If the FC accepts but nothing spins, check `MOT_PWM_TYPE` (see Flight controller settings), that the ESC is powered, and try `-t 10` or higher.

## Connecting a laptop GCS

In Mission Planner or QGroundControl, add a TCP link:

- Host: `mole.local` (or the Pi's IP address on your network; `10.42.0.1` in hotspot mode)
- Port: `5760`

That is, `tcp:mole.local:5760`. With MAVProxy: `mavproxy.py --master=tcp:mole.local:5760`.

Several clients can be connected at once; mavlink-router passes MAVLink between all of them and the FC.

## Field Wi-Fi: the MOLE hotspot

With no router around, the Pi can broadcast its own Wi-Fi network for the laptop to join. The Pi has one Wi-Fi radio, so it is either on the home network or running the hotspot, never both. You switch by hand with `mole-wifi`, which `setup.sh` installs to `/usr/local/bin`:

```bash
sudo mole-wifi status     # current mode, address, connected laptops, mode at boot
sudo mole-wifi hotspot    # before heading out: broadcast MOLE
sudo mole-wifi home       # back home: rejoin the saved home Wi-Fi
```

| | |
|---|---|
| SSID | `MOLE` |
| Password | `MOLEMOLE` (WPA2 needs 8+ characters) |
| Pi address | `10.42.0.1` (or `mole.local`) |
| Laptop address | handed out by the Pi, `10.42.0.x`; `sudo mole-wifi status` lists it |
| Band | 2.4 GHz, channel 6 |

- **The mode persists across reboots and battery swaps.** Switch to hotspot before you leave, while you can still reach the Pi at home.
- **Switching drops SSH over Wi-Fi.** The switch runs as a detached systemd unit (`mole-wifi-switch`) and finishes on its own. Reconnect on the new network: `ssh molepi@10.42.0.1` on MOLE, `ssh molepi@mole.local` at home.
- **It switches back if the new mode fails.** If the hotspot doesn't start, or no home network is found, within 30 s, it returns to the previous mode. So running `mole-wifi home` in the field leaves the hotspot running.
- If the Pi is unreachable in both modes, plug in Ethernet or a screen. The log of the last switch: `journalctl -u mole-wifi-switch`.
- **Internet on the laptop:** the hotspot gives clients no default route or DNS (`/etc/NetworkManager/dnsmasq-shared.d/mole-no-gateway.conf`). The laptop keeps using any other connection for the internet, e.g. iPhone USB tethering, and only talks to the Pi over MOLE.
- **telemetry_bridge.py** needs the laptop's hotspot address: `--laptop-ip 10.42.0.x` from `mole-wifi status`.
- The hotspot is the NetworkManager profile `MOLE-hotspot`. Change the password with `sudo nmcli con modify MOLE-hotspot wifi-sec.psk NEWPASS`. It takes effect on the next switch to hotspot.
- **Range:** the Pi's onboard antenna is small and sits next to the motors and ESC. Range-test on the ground before flying, and stay well within it.

Tested 2026-10-26: switched home → hotspot (`AP-ENABLED`, 10.42.0.1) → home over SSH, with no hands-on access needed.

## Git

`~/mole` is a clone of [JayChintala/Multirotor-Operated-Lab-Experiments-MOLE](https://github.com/JayChintala/Multirotor-Operated-Lab-Experiments-MOLE). The Pi files live in `companion/`.

- The Pi pushes using a GitHub **deploy key** (`~/.ssh/id_ed25519` on the Pi) with write access to this repo only.
- Commits are made as `JayChintala` with a GitHub no-reply email (set in the Pi's global git config), so no personal email is published.
- Run `git pull` and `git push` from `~/mole` on the Pi. From the Mac, run them over SSH: `ssh pi "cd mole && git status"`.
