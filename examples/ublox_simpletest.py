# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_ublox

# Create I2C bus using the board's default I2C pins
i2c = board.I2C()


# Create DDC transport for the u-blox module with default I2C address (0x42)
ddc = adafruit_ublox.UBloxDDC(i2c)


# Create a parser for handling NMEA sentences
gps = adafruit_ublox.GPS_UBloxI2C(ddc)


# Create a parser for handling UBX messages.
debug_ubx = False # Set to True to print raw UBX messages to the console
ubx = adafruit_ublox.UBloxUBX(ddc, debug=debug_ubx)


print("Configuring NMEA output (GGA and RMC only)...")
if ubx.set_nmea_output(gga=True, rmc=True) is not adafruit_ublox.UBX_SEND_OK:
    raise RuntimeError("Failed to configure NMEA output")

# Set 1Hz update rate
print("Setting 1 Hz update rate...")
if ubx.set_update_rate(1) is not adafruit_ublox.UBX_SEND_OK:
    raise RuntimeError("Failed to set update rate")


last_print = time.monotonic()
while True:
    gps.update()

    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        print("=" * 40)
        print(f"Lat: {gps.latitude:.6f} degrees, Lon: {gps.longitude:.6f} degrees")
        if gps.satellites is not None:
            print(f"# satellites: {gps.satellites}")
        if gps.altitude_m is not None:
            print(f"Altitude: {gps.altitude_m} meters")
        if gps.speed_knots is not None:
            print(f"Speed: {gps.speed_knots} knots")
        if gps.speed_kmh is not None:
            print(f"Speed: {gps.speed_kmh} km/h")
        if gps.track_angle_deg is not None:
            print(f"Track angle: {gps.track_angle_deg} degrees")
        if gps.horizontal_dilution is not None:
            print(f"Horizontal dilution: {gps.horizontal_dilution}")
        if gps.height_geoid is not None:
            print(f"Height geoid: {gps.height_geoid} meters")
