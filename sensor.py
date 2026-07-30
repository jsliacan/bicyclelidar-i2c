import time
from multiprocessing.connection import Connection

import smbus2 as smbus

from bicycleinit.BicycleSensor import BicycleSensor


def main(bicycleinit: Connection, name: str, args: dict):

    BUS = 1  # on RPi5 it's bus no.1. Check with `ls /dev/*i2c*`
    ADDRESS = 0x10  # get with `sudo i2cdetect-y 1` (install `i2c-tools` to have `i2cdetect`)
    DATA_CMD = [0x5A, 0x05, 0x00, 0x01, 0x60]  # TF-mini-S requires this trigger sequence to populate its register bank

    def get_data(b: smbus.SMBus, p: float):
        """
        Return reading triple from the lidar: (dist, strength, temp).

        b : bus to use (usually 1 on RPi5)
        p : sleep interval (usually 1/freq for a desired frequency)
        """

        # Instruct the sensor to dump its current frame into the I2C registers
        b.write_i2c_block_data(ADDRESS, 0x00, DATA_CMD)
        time.sleep(p)

        # Read 7 consecutive bytes directly starting from Register 0x00
        # Unlike for UART, no write instruction command is required.
        data = b.read_i2c_block_data(ADDRESS, 0x00, 9)

        # TFmini-S I2C Data Map (without UART emulation headers at 0 & 1):
        # data[0] = Dist_L, data[1] = Dist_H
        # data[2] = Strength_L, data[3] = Strength_H
        # data[4] = Temp_L, data[5] = Temp_H

        # Verify the UART emulation header bytes (0x59, 0x59) are present
        if data[0] == 0x59 and data[1] == 0x59:
            # Distance (Bytes 2 & 3)
            dist = data[2] | (data[3] << 8)

            # Signal Strength (Bytes 4 & 5)
            strength = data[4] | (data[5] << 8)

            # Chip Temperature (Bytes 6 & 7)
            temp_raw = data[6] | (data[7] << 8)
            temp = (temp_raw / 8) - 256
        else:
            # If the header shifts or returns zeros, throw an explicit error to catch it
            raise ValueError(f"Invalid frame header or empty buffer: {data[:2]}")

        return [dist, strength, temp]

    try:
        sensor = BicycleSensor(bicycleinit, name, args)

        sensor.write_header(["distance,strength,temperature"])

        actual_bus = smbus.SMBus(BUS)

        while True:
            pause = 1.0 / args.get("measurement_frequency", 50.0)
            sensor.write_measurement(get_data(actual_bus, pause))

    except:
        sensor.send_msg("Not able to instantiate bus (or some issue in the main loop)")

    finally:
        sensor.shutdown()


if __name__ == "__main__":
    main(None, "bicyclelidar", {"measurement_frequency": 50.0})
