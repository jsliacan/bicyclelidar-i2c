import time
from multiprocessing.connection import Connection

import smbus2 as smbus

from bicycleinit.BicycleSensor import BicycleSensor


def main(bicycleinit: Connection, name: str, args: dict):

    BUS = 1  # on RPi5 it's bus no.1. Check with `ls /dev/*i2c*`
    ADDRESS = 0x10  # get with `sudo i2cdetect -y 1` (install `i2c-tools` to have `i2cdetect`)
    CMD_HEADER = 0x5A  # header byte of every TFmini Plus command frame
    DATA_CMD = [
        0x05,
        0x00,
        0x01,
        0x60,
    ]  # TFmini Plus data request: frame length 5, cmd 0x00, param 0x01 (data in cm), checksum
    # NOTE: the TFmini Plus must have been switched to I2C mode beforehand (via the
    # SET_I2C_MODE UART command 5A 05 0A 01 60). The setting survives power cycles.

    def get_data(b: smbus.SMBus, p: float):
        """
        Return reading triple from the lidar: (dist, strength, temp).

        b : bus to use (usually 1 on RPi5)
        p : sleep interval (usually 1/freq for a desired frequency)
        """

        # Request one data frame (in cm). Unlike the TFmini-S, the TFmini Plus
        # expects the 0x5A header byte as the first byte on the wire, which in
        # smbus2 terms is the register/cmd byte of the block write.
        b.write_i2c_block_data(ADDRESS, CMD_HEADER, DATA_CMD)
        time.sleep(p)

        # The reply frame is 9 bytes:
        # [0]=0x59 [1]=0x59 (frame header)
        # [2]=Dist_L [3]=Dist_H
        # [4]=Strength_L [5]=Strength_H
        # [6]=Temp_L [7]=Temp_H
        # [8]=checksum (low byte of the sum of bytes 0..7)
        data = b.read_i2c_block_data(ADDRESS, 0x00, 9)

        # Verify the frame header bytes (0x59, 0x59) are present
        if data[0] == 0x59 and data[1] == 0x59:
            # Checksum: sum of all bytes except the last must match it
            if sum(data[:8]) & 0xFF != data[8]:
                raise ValueError(f"Checksum mismatch in data frame: {data}")

            # Distance (Bytes 2 & 3), in cm
            dist = data[2] | (data[3] << 8)

            # Signal Strength (Bytes 4 & 5)
            strength = data[4] | (data[5] << 8)

            # Chip Temperature (Bytes 6 & 7)
            temp_raw = data[6] | (data[7] << 8)
            temp = (temp_raw >> 3) - 256
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
