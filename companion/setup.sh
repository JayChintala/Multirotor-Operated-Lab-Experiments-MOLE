#!/usr/bin/env bash
# Set up a Raspberry Pi (Raspberry Pi OS / Debian trixie) as the MOLE MAVLink
# companion computer. Safe to re-run: every step checks before changing anything.
#
#   git clone https://github.com/JayChintala/Multirotor-Operated-Lab-Experiments-MOLE.git ~/mole
#   bash ~/mole/companion/setup.sh
#
# Run as the normal user (not root); it uses sudo where needed. It never reboots
# or touches SSH/Wi-Fi settings; it tells you when a reboot is required.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HOME/mav"
ROUTER_SRC="$HOME/src/mavlink-router"
CONFIG_TXT=/boot/firmware/config.txt
REBOOT_NEEDED=0

log() { printf '\n==> %s\n' "$*"; }

if [ "$(id -u)" -eq 0 ]; then
  echo "Run this as your normal user, not with sudo." >&2
  exit 1
fi

log "Packages"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Options::=--force-confold full-upgrade
sudo DEBIAN_FRONTEND=noninteractive apt-get -y install \
  git python3-venv python3-pip tmux meson ninja-build pkg-config gcc g++

log "UART on GPIO 14/15 (pins 8/10)"
# raspi-config nonint: get_serial_cons 1 = login console off; get_serial_hw 0 = UART on
if [ "$(sudo raspi-config nonint get_serial_cons)" != 1 ]; then
  sudo raspi-config nonint do_serial_cons 1
  REBOOT_NEEDED=1
fi
if [ "$(sudo raspi-config nonint get_serial_hw)" != 0 ]; then
  sudo raspi-config nonint do_serial_hw 0
  REBOOT_NEEDED=1
fi
# Give the full PL011 UART (ttyAMA0) to the GPIO pins instead of Bluetooth
if ! grep -qE '^[[:space:]]*dtoverlay=disable-bt([[:space:]]|$)' "$CONFIG_TXT"; then
  printf '\n[all]\ndtoverlay=disable-bt\n' | sudo tee -a "$CONFIG_TXT" >/dev/null
  REBOOT_NEEDED=1
fi
# hciuart only exists on older images
if systemctl cat hciuart.service >/dev/null 2>&1; then
  sudo systemctl disable --now hciuart
fi
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  REBOOT_NEEDED=1
fi

log "Python venv ($VENV) with pymavlink + MAVProxy"
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
"$VENV/bin/pip" install --quiet pymavlink MAVProxy
grep -qxF 'source ~/mav/bin/activate' ~/.bashrc ||
  printf '\n# MAVLink venv\nsource ~/mav/bin/activate\n' >> ~/.bashrc

log "mavlink-router (build from source)"
if [ ! -d "$ROUTER_SRC/.git" ]; then
  mkdir -p "$(dirname "$ROUTER_SRC")"
  git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git "$ROUTER_SRC"
fi
git -C "$ROUTER_SRC" submodule update --init --recursive
[ -d "$ROUTER_SRC/build" ] || meson setup "$ROUTER_SRC/build" "$ROUTER_SRC" --buildtype=release
ninja -C "$ROUTER_SRC/build" -j2   # -j2: the Pi 3B+ has ~1 GB RAM
sudo ninja -C "$ROUTER_SRC/build" install

log "mavlink-router config + service"
sudo install -d /etc/mavlink-router
CONF_CHANGED=0
if ! sudo cmp -s "$REPO_DIR/mavlink-router/main.conf" /etc/mavlink-router/main.conf; then
  if [ -f /etc/mavlink-router/main.conf ]; then
    sudo cp /etc/mavlink-router/main.conf /etc/mavlink-router/main.conf.bak
  fi
  sudo install -m 644 "$REPO_DIR/mavlink-router/main.conf" /etc/mavlink-router/main.conf
  CONF_CHANGED=1
fi
sudo systemctl daemon-reload
sudo systemctl enable mavlink-router
if [ "$CONF_CHANGED" = 1 ]; then
  sudo systemctl restart mavlink-router
else
  sudo systemctl start mavlink-router   # no-op if already running
fi

log "Checks"
chmod +x "$REPO_DIR/check_link.py"
sleep 2
if ss -ltn | grep -q ':5760 '; then
  echo "OK   mavlink-router is listening on TCP 5760"
else
  echo "FAIL mavlink-router is not listening on 5760: journalctl -u mavlink-router -n 50"
fi

if [ "$REBOOT_NEEDED" = 1 ]; then
  echo
  echo "REBOOT NEEDED for the UART/group changes: sudo reboot"
  echo "Then re-run this script to confirm everything shows OK."
  exit 0
fi

if [ "$(readlink /dev/serial0 2>/dev/null)" = ttyAMA0 ]; then
  echo "OK   /dev/serial0 -> ttyAMA0"
else
  echo "FAIL /dev/serial0 -> $(readlink /dev/serial0 2>/dev/null || echo missing) (expected ttyAMA0)"
fi
if systemctl is-active --quiet serial-getty@ttyAMA0; then
  echo "FAIL a login console (getty) is running on ttyAMA0"
else
  echo "OK   no login console on ttyAMA0"
fi

echo
echo "Done. With the flight controller wired up, run: $VENV/bin/python $REPO_DIR/check_link.py"
echo "Bench motor test (PROPS OFF): $VENV/bin/python $REPO_DIR/motor_test.py"
