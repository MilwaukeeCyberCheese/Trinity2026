import neopixel
import itertools
from neopixel import NeoPixel
from typing import Optional
from time import sleep
import argparse
from networktables import NetworkTables
from time import time
import board

C_NOCONN = 0xFFFFFF
# orange
C_ESTOP = 0xF4A524

C_RED = 0xFF0000
C_BLUE = 0x0000FF

# teal
C_FMS = 0x77EABC
# purple
C_PHOTON = 0x7839DD
# yellow
C_CAN = 0xC1ED3D


AKIT_CONN_KEYS = [
    "Drive/Gyro",
    "Hopper",
    "Intake",
    "IntakeRoller",
    "Loader",
]

for i in range(4):
    for c in ["drive", "turn"]:
        AKIT_CONN_KEYS.append(f"Drive/{i}/{c}")


class CCNeoPixel:
    inner: NeoPixel

    def __init__(self, pin, count: int):
        self.inner = NeoPixel(pin, count, auto_write=False, pixel_order=neopixel.RGB)

    def set_all(self, color: int):
        self.inner.fill(color)
        self.inner.show()

    def set_cycle(self, colors: list[int]):
        it = itertools.cycle(colors)

        for i in range(self.inner.n):
            self.inner[i] = next(it)

        self.inner.show()


class HeartbeatSubscriber:
    last: int
    time: float

    def __init__(self):
        self.last = -1
        self.time = -1

    def update(self, value: Optional[int]) -> bool:
        new_time = time()

        if value is None:
            self.last = -1
            self.time = -1
            return False

        if self.last != value:
            self.last = value
            self.time = new_time

            return True

        return new_time > self.time + 0.5


def run_nt(neopixel: CCNeoPixel):
    cameras = {
        "FrontThrifty": HeartbeatSubscriber(),
        "BackThrifty": HeartbeatSubscriber(),
    }

    fms_info = NetworkTables.getTable("FMSInfo")
    pv = NetworkTables.getTable("photonvision")
    akit = NetworkTables.getTable("AdvantageKit")
    ds = NetworkTables.getTable("AdvantageKit/DriverStation")
    i = 0

    while True:
        sleep(0.1)

        if not NetworkTables.isConnected():
            print("noconn!")
            neopixel.set_all(C_NOCONN)
            continue

        estop = ds.getBoolean("EmergencyStop", False)

        if estop:
            print("estop!")
            neopixel.set_all(C_ESTOP)
            continue

        is_red = fms_info.getBoolean("IsRedAlliance", False)
        fms_connected = pv.getBoolean("FMSAttached", False)

        pv_connected = True
        for name, hb in cameras.items():
            connected = hb.update(pv.getNumber(name + "/heartbeat", None))
            pv_connected &= connected

        can_connected = True
        for k in AKIT_CONN_KEYS:
            connected = akit.getBoolean(k + "/connected", False)
            can_connected &= connected

        team_color = C_RED if is_red else C_BLUE

        colors = [team_color]
        if not fms_connected:
            colors.append(C_FMS)
        if not pv_connected:
            colors.append(C_PHOTON)
        if not can_connected:
            colors.append(C_CAN)

        if i % 10 == 0:
            print(colors)

        neopixel.set_cycle(colors)
        i += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", type=str, help="IP address to connect to")
    args = parser.parse_args()

    NetworkTables.initialize(server=args.ip)

    np = CCNeoPixel(board.D18, 64)

    run_nt(np)


if __name__ == "__main__":
    main()
