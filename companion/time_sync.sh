#!/usr/bin/env bash
# Keep the Pi's clock locked to the ground-station laptop with chrony, so logs from the
# laptop (ROS/MAVROS/Vicon), the Pi (kill switch, signal trace) and the FC line up.
# The Pi has no battery-backed clock and the MOLE hotspot has no internet, so without this
# its clock is wrong after every boot.
#
#   On the laptop (once):  bash companion/time_sync.sh server [subnet]   # default 10.42.0.0/24
#   On the Pi (once):      bash companion/time_sync.sh client <laptop-ip-or-hostname>
#   Check either side:     bash companion/time_sync.sh status
#
# chrony (not ntpdate): ntpdate sets the clock once and is deprecated; chrony keeps it locked
# continuously and steps it at boot. Clock sync does not lower latency; it makes timestamps
# comparable so latency can be measured. The FC gets the laptop's time from MAVROS (SYSTEM_TIME).
# Safe to re-run.
set -euo pipefail

CONF=/etc/chrony/conf.d/mole.conf
SOURCES=/etc/chrony/sources.d/mole.sources

need_chrony() {
  if ! command -v chronyd >/dev/null; then
    # Replaces systemd-timesyncd (they cannot run together).
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y install chrony
  fi
  sudo mkdir -p /etc/chrony/conf.d /etc/chrony/sources.d
  # Older chrony.conf files lack these includes.
  grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf ||
    echo 'confdir /etc/chrony/conf.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
  grep -q '^sourcedir /etc/chrony/sources.d' /etc/chrony/chrony.conf ||
    echo 'sourcedir /etc/chrony/sources.d' | sudo tee -a /etc/chrony/chrony.conf >/dev/null
}

case "${1:-}" in
  server)
    subnet="${2:-10.42.0.0/24}"
    need_chrony
    sudo tee "$CONF" >/dev/null <<EOF
# MOLE: serve time to the Pi. Keeps serving from the local clock when there is no internet.
allow $subnet
local stratum 10
EOF
    sudo systemctl restart chrony
    echo "Laptop serves time to $subnet. If a firewall is on: sudo ufw allow from $subnet to any port 123 proto udp"
    ;;
  client)
    laptop="${2:?usage: $0 client <laptop-ip-or-hostname>}"
    need_chrony
    sudo tee "$SOURCES" >/dev/null <<EOF
# MOLE: follow the ground-station laptop (polled every 4-16 s).
server $laptop iburst prefer minpoll 2 maxpoll 4
EOF
    sudo tee "$CONF" >/dev/null <<EOF
# MOLE: step (jump) the clock whenever it is more than 0.5 s off, not only at boot:
# the Pi boots with a stale clock and may only reach the laptop later.
makestep 0.5 -1
EOF
    sudo systemctl restart chrony
    echo "Pi follows $laptop. Check in a minute with: bash $0 status"
    ;;
  status)
    chronyc -n tracking
    chronyc -n sources -v
    ;;
  *)
    sed -n '2,15p' "$0"
    exit 1
    ;;
esac
