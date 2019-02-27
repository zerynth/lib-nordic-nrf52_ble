################################################################################
# BLE Scanner
#
# Created by Zerynth Team 2019 CC
# Author: G. Baldi
###############################################################################

import streams
# import a BLE driver: in this example we use NRF52
from nordic.nrf52_ble import nrf52_ble as bledrv
# then import the BLE modue
from wireless import ble


streams.serial()


# Let's define some callbacks and constants

# How long to scan for in milliseconds
scan_time=30000

def scan_report_cb(data):
    print("Detected packet from",ble.btos(data[4]),"containing",ble.btos(data[3]))
    print("         packet is of type",ble.btos(data[0]),"while address is of type",ble.btos(data[1]))
    print("         remote device has RSSI of",ble.btos(data[2]))

def scan_start_cb(data):
    print("Scan started")

def scan_stop_cb(data):
    print("Scan stopped")
    #let's start it up again
    ble.start_scanning(scan_time)

try:
    # initialize BLE driver
    bledrv.init()

    # Set GAP name and no security
    ble.gap("Zerynth",security=(ble.SECURITY_MODE_1,ble.SECURITY_LEVEL_1))

    ble.add_callback(ble.EVT_SCAN_REPORT,scan_report_cb)
    ble.add_callback(ble.EVT_SCAN_STARTED,scan_start_cb)
    ble.add_callback(ble.EVT_SCAN_STOPPED,scan_stop_cb)

    #set scanning parameters: every 100ms for 50ms and no duplicates
    ble.scanning(100,50,duplicates=0)

    # Start the BLE stack
    ble.start()

    # Now start scanning for 30 seconds
    ble.start_scanning(scan_time)

except Exception as e:
    print(e)

# loop forever
while True:
    print(".")
    sleep(10000)

