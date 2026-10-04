# MOLE: MAVLink companion computer

Raspberry Pi 3B+ (`MOLE`, Debian 13 trixie, 64-bit), running as a MAVLink companion
computer for an ArduPilot flight controller (TBS Lucid H7).

```
FC (SERIAL7, 921600) ──UART──> /dev/serial0 ──> mavlink-router ──> TCP :5760
                                                                    ├─ check_link.py / local scripts
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

## Wiring (FC not connected yet)

| Pi pin          | Lucid H7 |
|-----------------|----------|
| 8  (GPIO14 TXD) | RX7      |
| 10 (GPIO15 RXD) | TX7      |
| 6  (GND)        | GND      |

FC parameters: `SERIAL7_PROTOCOL=2` (MAVLink2), `SERIAL7_BAUD=921`. Reboot the FC after setting them.

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

## Connecting a laptop GCS

In Mission Planner or QGroundControl, add a TCP link:

- Host: `mole.local` (or the Pi's IP address on your network)
- Port: `5760`

That is, `tcp:mole.local:5760`. With MAVProxy: `mavproxy.py --master=tcp:mole.local:5760`.

Several clients can be connected at once; mavlink-router passes MAVLink between all of them and the FC.

## Git

`~/mole` is a clone of [JayChintala/Multirotor-Operated-Lab-Experiments-MOLE](https://github.com/JayChintala/Multirotor-Operated-Lab-Experiments-MOLE). The Pi files live in `companion/`.

- The Pi pushes using a GitHub **deploy key** (`~/.ssh/id_ed25519` on the Pi) with write access to this repo only.
- Commits are made as `JayChintala` with a GitHub no-reply email (set in the Pi's global git config), so no personal email is published.
- Run `git pull` and `git push` from `~/mole` on the Pi. From the Mac, run them over SSH: `ssh pi "cd mole && git status"`.
