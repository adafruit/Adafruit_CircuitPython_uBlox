# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2026 Limor Fried/Ladyada for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

import time
import board
import adafruit_ublox

# Create I2C bus and u-blox DDC transport
i2c = board.I2C()
ddc = adafruit_ublox.UBloxDDC(i2c)

# Create UBX protocol engine for configuration
ubx = adafruit_ublox.UBloxUBX(ddc, debug=True)

# Allow module to start up
time.sleep(1)

# Configure which NMEA sentences to output (GGA and RMC only)
print("Configuring NMEA output...")
ubx.set_nmea_output(gga=True, rmc=True, gll=False, gsa=False, gsv=False, vtg=False)

# Set 1 Hz update rate
print("Setting 1 Hz update rate...")
ubx.set_update_rate(1)

# Create GPS NMEA parser using the DDC transport
gps = adafruit_ublox.GPS_UBloxI2C(ddc)

last_print = time.monotonic()
while True:
    gps.update()

    current = time.monotonic()
    if current - last_print >= 2.0:
        last_print = current
        if not gps.has_fix:
            print("Waiting for fix...")
            continue
        print("=" * 40)
        print(
            "Lat: {:.6f} degrees, Lon: {:.6f} degrees".format(
                gps.latitude, gps.longitude
            )
        )
        if gps.altitude_m is not None:
            print("Altitude: {} meters".format(gps.altitude_m))
        if gps.satellites is not None:
            print("Satellites: {}".format(gps.satellites))
        if gps.speed_knots is not None:
            print("Speed: {} knots".format(gps.speed_knots))
