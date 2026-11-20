# MOLE: MAVLink companion computer

Raspberry Pi 3B+ (`MOLE`, Debian 13 trixie, 64-bit), running as a MAVLink companion
computer for an ArduPilot flight controller (TBS Lucid H7).

```
FC (SERIAL7, 921600) ──UART──> /dev/serial0 ──> mavlink-router ──> TCP :5760
                                                                    ├─ check_link.py / motor_test.py
                                                                    ├─ kill_switch.py or manual_kill_switch.py <── UDP 5002 ── laptop control center
                                                                    ├─ telemetry_bridge.py ── UDP 5001 ──> laptop control center
                                                                    └─ laptop GCS
```

## Quick setup (fresh Pi)

Everything in "As-built reference" below is automated by `setup.sh`. It is safe to re-run.

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
| `SERVO1_FUNCTION`–`SERVO4_FUNCTION` | 36, 33, 34, 35 | Output 1 = motor 4, output 2 = motor 1, output 3 = motor 2, output 4 = motor 3 (from FC log 00016) |
| `BRD_SAFETY_DEFLT` | 0 | No safety switch |
| `SERVO_DSHOT_ESC` | 1 | Lets the FC send DShot commands to the ESC (BLHeli32/AM32 type). Without it, `SERVO_BLH_RVMASK` has no effect |
| `SERVO_BLH_MASK` | 15 | Outputs 1–4 are DShot ESCs managed by the FC |
| `SERVO_BLH_RVMASK` | 9 | Reverses outputs 1 and 4, which with the mapping above are motors 4 and 3. The resulting spin directions were checked motor by motor with `motor_test.py` and match Quad X: front-right and rear-left CCW, front-left and rear-right CW (seen from above) |
| `FS_GCS_ENABLE` | 5 | Land if the GCS link is lost. With no RC receiver, the Wi-Fi link to QGC is the only control link |
| `FS_GCS_TIMEOUT` | 5 | Seconds without a GCS heartbeat before the failsafe triggers |
| `FS_OPTIONS` | 0 | GCS failsafe applies in every mode. The earlier value, 18, kept flying in Stabilize/AltHold/Loiter and in Auto missions after link loss |

Reboot the FC after changing `SERIAL7_*` or `MOT_PWM_TYPE`. With DShot working, the ESC plays its start-up tones followed by confirmation beeps once the FC has booted.

Bench notes:
- The motors spin on a 12 V bench supply (10 A limit). Expect lower top speed than on 6S.
- No RC receiver is connected yet, so the FC reports 0 RC channels.
- `BATT_LOW_VOLT` is 10.5 V (a 3S value) and the FC reports the battery monitor as unhealthy. Set the battery parameters for 6S before flying.

## As-built reference

`setup.sh` is the source of truth; this is what it leaves on the Pi.

- **Packages:** git, python3-venv, python3-pip, tmux, meson, ninja-build, pkg-config, gcc, g++.
- **UART:** serial login console off (`raspi-config nonint do_serial_cons 1`), UART hardware on (`enable_uart=1`), and `dtoverlay=disable-bt` in `/boot/firmware/config.txt`, so `/dev/serial0 -> ttyAMA0` on GPIO 14/15. `serial-getty@ttyAMA0` is disabled. The user is in `dialout`.
- **Python:** venv at `~/mav` with `pymavlink` and `MAVProxy`; `~/.bashrc` activates it.
- **mavlink-router:** built from source in `~/src/mavlink-router` and installed to `/usr/bin/mavlink-routerd` as the enabled `mavlink-router` service. Config `/etc/mavlink-router/main.conf` (copy in `companion/mavlink-router/main.conf`): UART endpoint `fc` on `/dev/serial0` at 921600, TCP server on port 5760.
- **Services and tools:** `kill-switch` service (`kill-switch.service` → `kill_switch.py`), and `mole-wifi` in `/usr/local/bin`.

### Scripts

| File | What it does |
|---|---|
| `setup.sh` | Reproduces the whole setup on a fresh Pi (see Quick setup) |
| `check_link.py` | Read-only link check against mavlink-router (see Checking the link) |
| `motor_test.py` | Bench motor test, one motor at a time (see Motor test) |
| `kill_switch.py` | Watchdog kill receiver on UDP 5002, run by `kill-switch.service` (see Kill switch) |
| `manual_kill_switch.py` | Manual-only bench kill receiver on UDP 5002; KILL latches motor-test cancel + force-disarm until reset (see [BENCH_KILL.md](BENCH_KILL.md)) |
| `telemetry_bridge.py` | Read-only bridge: FC telemetry (attitude, IMU, position, battery, mode, arming, motor outputs, ESC RPM) to the laptop backend on UDP 5001 |
| `signal_trace.py` | Shared module that sends best-effort signal records to the laptop's Signal Monitor; never gates control |
| `test_signal_trace.py` | Unit tests for `signal_trace.py` (`python -m unittest test_signal_trace` from `companion/`) |
| `mavlink-router/main.conf` | Copy of `/etc/mavlink-router/main.conf`. After editing: `sudo install -m 644 companion/mavlink-router/main.conf /etc/mavlink-router/` and restart the service |
| `kill-switch.service`, `mole-wifi` | Installed by `setup.sh` |

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

## Kill switch

There are two kill receivers. Both listen on UDP 5002 for packets from the laptop backend, so **only one can run at a time**.

| | `kill_switch.py` (watchdog) | `manual_kill_switch.py` (bench) |
|---|---|---|
| Started by | `kill-switch` service, at boot | By hand, see [BENCH_KILL.md](BENCH_KILL.md) |
| Stops on KILL | Yes: force-disarm | Yes: motor-test cancel + force-disarm, latched until reset |
| Stops on laptop link loss | Yes, if armed and no heartbeat for 1 s | No |
| Use for | Flying | Props-off bench tests with `motor_test.py` |

To switch to the bench receiver: `sudo systemctl stop kill-switch`. Start it again with `sudo systemctl start kill-switch` before flying.

### Watchdog (`kill_switch.py`)

It **force-disarms the FC, which stops the props even in flight, so the drone drops**. Only fly where that is safe, such as the padded cage. It disarms when:

- **KILL** is pressed in the control center, or
- **the laptop link drops** while the FC is armed: no heartbeat from the control center backend for 1 s (`--timeout`). The backend sends the heartbeat only while a control center page is open. Closing the page, stopping the backend, the laptop sleeping or the Wi-Fi dropping all stop the props.

It also disarms an FC that is **armed with no control center connected**, so start the backend and open the page before arming.

- The disarm is `MAV_CMD_COMPONENT_ARM_DISARM` with param2 = 21196 (forced), repeated every 0.1 s until the FC reports disarmed. The FC accepts it (tested on the bench).
- It runs on the Pi and talks to the FC through mavlink-router over the UART, so it works with the Wi-Fi down. If the Pi itself fails, the FC's own GCS failsafe (`FS_GCS_ENABLE` = 5) lands after 5 s.
- QGC flying the drone isn't watched. If only QGC disconnects, the FC's GCS failsafe lands it.
- The control center shows the kill receiver's status. Check it is fresh before every arming.

```bash
sudo systemctl status kill-switch     # running?
journalctl -u kill-switch -f          # link up/lost, KILL and disarm events
```

**Test it before flying, with the props off:** arm, then (1) press KILL and (2) re-arm and close the control center tab. The FC should disarm within about 1 s each time.

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

## Pre-flight checklist

Before the next real flight. Simulator prerequisites (model, initial tune, gain sweep, parameter file) are in [`ROS/sim`](../ROS/sim/README.md).

**Decide**
1. **Per-motor current limit (`MOT_SPIN_MAX`)** from the motor and ESC datasheets (continuous ratings). At 0.73 each motor is capped at 2 kgf / 34.5 A.
2. **RC transmitter and receiver.** Optional for a tethered hover, strongly recommended before free flight, essential before AutoTune. Wi-Fi is currently the only control link.

**Hardware**
3. **Reduce vibration:** soft-mount the FC, balance props, tighten everything (the crash log had 114 clipping events).
4. **Props and motors:** correct CW/CCW prop on each motor, no damage, bells spin freely.
5. **Battery:** fully charged; check the voltage reading against a multimeter (`BATT_VOLT_MULT`). Set the 6S battery parameters.

**FC setup**
6. **Back up the FC parameters** (Mission Planner: Config → Full Parameter List → Save).
7. **Load the new parameter file** from the simulator work and reboot.
8. **Recalibrate accelerometer and level.** Check `AHRS_ORIENTATION` is still 8 (FC mounted upside down).
9. **Bench checks, props off:** `motor_test.py` for order and spin; tilt by hand and check the horizon in the ground station; confirm the GCS failsafe triggers when Wi-Fi drops; test the kill switch (above).

**First flight**
10. **Short, low hover:** tethered or in the cage, AltHold, 30–60 s at about 1 m. Be ready to switch to LAND or disarm; stop at the first wobble.
11. **Analyse the log:** oscillation, vibration, hover throttle; compare with the simulator and calibrate the model.

## Git

`~/mole` is a clone of [JayChintala/Multirotor-Operated-Lab-Experiments-MOLE](https://github.com/JayChintala/Multirotor-Operated-Lab-Experiments-MOLE). The Pi files live in `companion/`.

- The Pi pushes using a GitHub **deploy key** (`~/.ssh/id_ed25519` on the Pi) with write access to this repo only.
- Commits are made as `JayChintala` with a GitHub no-reply email (set in the Pi's global git config), so no personal email is published.
- Run `git pull` and `git push` from `~/mole` on the Pi.
