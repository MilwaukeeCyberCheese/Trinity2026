#!/bin/bash

set -euo pipefail

PI_HOST="${PI_HOST:-10.88.47.69}"
PI_USER="${PI_USER:-pi}"
PI_PASSWORD="${PI_PASSWORD:-password}"
APP_NAME="${APP_NAME:-whitesands}"
APP_DIR="${APP_DIR:-/home/${PI_USER}/${APP_NAME}}"
SERVICE_NAME="${SERVICE_NAME:-${APP_NAME}.service}"
ARCHIVE_NAME="${ARCHIVE_NAME:-${APP_NAME}.tar.gz}"

if [[ $# -ne 1 ]]; then
	echo "Usage: $0 <nt-server-ip>"
	exit 1
fi

NT_SERVER_IP="$1"
ARCHIVE="/tmp/${ARCHIVE_NAME}"

if ! command -v sshpass >/dev/null 2>&1; then
	echo "sshpass is required to deploy non-interactively"
	exit 1
fi

./build.sh

tar --mtime='2024-01-01 00:00:00Z' -czf "$ARCHIVE" -C builddir .

sshpass -p "$PI_PASSWORD" ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_HOST}" "mkdir -p '${APP_DIR}'"
sshpass -p "$PI_PASSWORD" scp -o StrictHostKeyChecking=no "$ARCHIVE" "${PI_USER}@${PI_HOST}:${APP_DIR}/"

sshpass -p "$PI_PASSWORD" ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_HOST}" \
	"APP_DIR='${APP_DIR}' SERVICE_NAME='${SERVICE_NAME}' NT_SERVER_IP='${NT_SERVER_IP}' PI_USER='${PI_USER}' PI_PASSWORD='${PI_PASSWORD}' ARCHIVE_NAME='${ARCHIVE_NAME}' bash -s" <<'EOF'
set -euo pipefail

cd "$APP_DIR"
tar -xzf "$ARCHIVE_NAME"
rm -f "$ARCHIVE_NAME"

chmod +x setup.sh
./setup.sh

cat >/tmp/"$SERVICE_NAME" <<SERVICE
[Unit]
Description=White Sands LED service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=$APP_DIR
ExecStart=$APP_DIR/.venv/bin/python $APP_DIR/main.py $NT_SERVER_IP
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
SERVICE

printf '%s\n' "$PI_PASSWORD" | sudo -S mv /tmp/"$SERVICE_NAME" /etc/systemd/system/"$SERVICE_NAME"
printf '%s\n' "$PI_PASSWORD" | sudo -S systemctl daemon-reload
printf '%s\n' "$PI_PASSWORD" | sudo -S systemctl enable "$SERVICE_NAME"
printf '%s\n' "$PI_PASSWORD" | sudo -S systemctl restart "$SERVICE_NAME"
EOF
