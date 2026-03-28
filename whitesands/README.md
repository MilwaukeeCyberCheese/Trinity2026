# White Sands

Light controller for the robot LEDs running on a Raspberry Pi.

## What It Does

The script connects to NetworkTables, watches FMS, PhotonVision, and AdvantageKit status, and maps those states to LED colors on a NeoPixel strip.

## Files

- `main.py`: LED controller runtime.
- `build.sh`: Downloads all Python wheels needed for offline install on the Pi.
- `setup.sh`: Creates a venv on the Pi and installs from the offline wheel bundle.
- `deploy.sh`: Builds, uploads, installs, and configures the Pi service over SSH/SCP.

## Prerequisites

On the development machine:

- `python3`
- `pip`
- `sshpass`
- internet access when running `build.sh` or `deploy.sh`

On the Raspberry Pi:

- reachable at `10.88.47.69`
- SSH enabled
- user `pi`
- password `password`
- `sudo` access for the `pi` user
- `python3` and `python3-venv` installed
- a working C toolchain for building `sysv_ipc` and `rpi-ws281x` offline (`gcc`, Python headers, and standard build tools)
- the Raspberry Pi OS `RPi.GPIO` package installed on the system Python

## Offline Bundle Build

Build the wheel bundle and deployment files:

```bash
./build.sh
```

This creates `builddir/` with:

- `main.py`
- `setup.sh`
- `requirements.txt`
- `wheeldir/`

## Deploy To The Pi

Deploy and set the NetworkTables server IP used by the service:

```bash
./deploy.sh <nt-server-ip>
```

Example:

```bash
./deploy.sh 10.88.47.2
```

`deploy.sh` will:

1. run `./build.sh`
2. copy the bundle to `/home/pi/whitesands`
3. bootstrap `setuptools`/`wheel` offline, then run the offline install on the Pi, reusing system `RPi.GPIO` and building `sysv_ipc` and `rpi-ws281x` from source when needed
4. install a `systemd` service named `whitesands.service`
5. enable and restart the service

The service runs as `root` because the NeoPixel backend needs access to `/dev/mem`.

Required system packages on Raspberry Pi OS:

```bash
sudo apt install python3-rpi.gpio python3-dev build-essential
```

## Service Management

On the Pi:

```bash
sudo systemctl status whitesands.service
sudo systemctl restart whitesands.service
journalctl -u whitesands.service -f
```

## Notes

- The current runtime uses `pynetworktables`, so it speaks NT3, not NT4.
- If the Pi hostname, username, password, or install path changes, override them with environment variables when running `deploy.sh`.

Example:

```bash
PI_HOST=10.88.47.70 PI_PASSWORD=secret ./deploy.sh 10.88.47.2
```
