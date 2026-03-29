import neopixel
from neopixel import NeoPixel
from typing import Optional
from time import sleep, time
import argparse
from networktables import NetworkTables
import board

C_NOCONN = 0x000000
# orange
C_ESTOP = 0xF4A524

C_RED = 0xFF0000
C_BLUE = 0x0000FF

# teal
C_FMS = 0x77EABC
# purple
C_PHOTON = 0x690ef9
# yellow
C_CAN = 0xFFFF00


AKIT_CONN_KEYS =[
    "Drive/Gyro",
    "Hopper",
    "Intake",
    "IntakeRoller",
    "Loader",
]

for i in range(4):
    for c in["drive", "turn"]:
        AKIT_CONN_KEYS.append(f"Drive/{i}/{c}")


# Helper function to generate smooth RGB colors for the rainbow
def colorwheel(pos: int):
    pos = int(pos) % 256
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)


class CCNeoPixel:
    inner: NeoPixel

    def __init__(self, pin, count: int):
        self.inner = NeoPixel(pin, count, auto_write=False, pixel_order=neopixel.GRB)

    def set_all(self, color: int):
        print("zv: " + hex(color))

        self.inner.fill(color)
        self.inner.show()

    def set_cycle(self, colors: list[int]):
        color_idx = int(time() * 2) % len(colors)
        active_color = colors[color_idx]

        print("flash: " + hex(active_color) + " (cycle: " + ", ".join([hex(c) for c in colors]) + ")")

        self.inner.fill(active_color)
        self.inner.show()

    def set_rainbow(self):
        print("gaming mode")
        offset = int(time() * 150) % 256
        
        for i in range(self.inner.n):
            pixel_index = int((i * 256 / self.inner.n) + offset)
            self.inner[i] = colorwheel(pixel_index)
            
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

    while True:
        sleep(0.1)

        if not NetworkTables.isConnected():
            neopixel.set_all(C_NOCONN)
            continue

        fms_info = NetworkTables.getTable("FMSInfo")
        pv = NetworkTables.getTable("photonvision")
        akit = NetworkTables.getTable("AdvantageKit")
        ds = NetworkTables.getTable("AdvantageKit/DriverStation")
        sd = NetworkTables.getTable("SmartDashboard")

        estop = ds.getBoolean("EmergencyStop", False)

        if estop:
            neopixel.set_all(C_ESTOP)
            continue

        is_red = fms_info.getBoolean("IsRedAlliance", False)
        fms_connected = ds.getBoolean("FMSAttached", False)
        
        gaming_mode = sd.getBoolean("GamingModeActive", False)

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

        if len(colors) == 1 and gaming_mode:
            neopixel.set_rainbow()
        else:
            neopixel.set_cycle(colors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", type=str, help="IP address to connect to")
    args = parser.parse_args()

    NetworkTables.setNetworkIdentity("whitesands")
    NetworkTables.initialize(server=args.ip)

    np = CCNeoPixel(board.D18, 64)

    run_nt(np)


if __name__ == "__main__":
    main()