import time
from multiprocessing.connection import Connection

import smbus2 as smbus

from bicycleinit.BicycleSensor import BicycleSensor


def main(bicycleinit: Connection, name: str, args: dict):

    BUS = 1  # on RPi5 it's bus no.1. Check with `ls /dev/*i2c*`
    ADDRESS = 0x10  # get with `sudo i2cdetect-y 1` (install `i2c-tools` to have `i2cdetect`)
    DATA_CMD = [0x5A, 0x05, 0x00, 0x01, 0x60]  # distance instruction

    def get_data(b: smbus.SMBus, p: float):
        """
        Return reading triple from the lidar: (dist, strength, temp).

        b : bus to use (usually 1 on RPi5)
        p : sleep interval (1/freq for a desired frequency)
        """
        b.write_i2c_block_data(ADDRESS, 0x00, DATA_CMD)
        time.sleep(p)  # pause for 1/freq time
        data = b.read_i2c_block_data(ADDRESS, 0x00, 9)
        dist = data[0] | (data[1] << 8)
        strength = data[2] | (data[3] << 8)
        temp = (data[4] | (data[5] << 8)) / 100

        return [dist, strength, temp]

    try:
        actual_bus = smbus.SMBus(BUS)

        sensor = BicycleSensor(bicycleinit, name, args)

        sensor.write_header(["distance", "strength,temperature"])

        while True:
            pause = 1.0 / args.get("measurement_frequency", 50.0)
            sensor.write_measurement(get_data(actual_bus, pause))

    except:
        sensor.send_msg("Not able to instantiate bus (or some other issue in the main loop)")

    finally:
        sensor.shutdown()


if __name__ == "__main__":
    main(None, "bicyclelidar", {"measurement_frequency": 50.0})
