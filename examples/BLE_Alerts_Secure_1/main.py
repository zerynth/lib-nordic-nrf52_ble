################################################################################
# BLE Alerts with security
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


notifications_enabled = True
connected = False

# Let's define some callbacks
def value_cb(status,val):
    # check incoming commands and enable/disable notifications
    global notifications_enabled
    print("Value changed to",val[0],val[1])
    if val[0]==0:
        print("Notifications enabled")
        notifications_enabled = True
    elif val[0]==2:
        notifications_enabled = False
        print("Notifications disabled")
    else:
        print("Notifications unchanged")
        
def connection_cb(address):
    global connected
    print("Connected to",ble.btos(address))
    connected = True

def disconnection_cb(address):
    global connected
    print("Disconnected from",ble.btos(address))
    # let's start advertising again
    ble.start_advertising()
    connected = False

# Let's define some security callbacks
def show_key_cb(passkey):
    print("ENTER THIS PIN ON THE MASTER:",passkey)


try:
    # initialize BLE driver
    bledrv.init()

    # Set GAP name and LEVEL 2 security
    # !!! If security is not set, no secure connection will be possible
    ble.gap("ZNotifier",security=(ble.SECURITY_MODE_1,ble.SECURITY_LEVEL_2))

    # add some GAP callbacks
    ble.add_callback(ble.EVT_CONNECTED,connection_cb)
    ble.add_callback(ble.EVT_DISCONNECTED,disconnection_cb)
    
    # Create a GATT Service: let's try an Alert Notification Service
    # (here are the specs: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.alert_notification.xml)
    s = ble.Service(0x1811)

    # The Alert Notification service has multiple characteristics. Let's add them one by one
    
    # Create a GATT Characteristic for counting new alerts.
    # specs: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.supported_new_alert_category.xml
    cn = ble.Characteristic(0x2A47, ble.NOTIFY | ble.READ,16,"New Alerts",ble.BYTES)
    # Add the GATT Characteristic to the Service
    s.add_characteristic(cn)
    

    # Create anothr GATT Characteristic for enabling/disabling alerts
    # specs: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.alert_notification_control_point.xml
    cc = ble.Characteristic(0x2A44, ble.WRITE ,2,"Alerts control",ble.BYTES)
    # Add the GATT Characteristic to the Service
    s.add_characteristic(cc)
    # Add a callback to be notified of changes
    cc.set_callback(value_cb)
    
    # Add the Service. You can create additional services and add them one by one
    ble.add_service(s)

    # Configure security. BLE security is very flexible.
    # In this case we declare that the device has only an output capability (CAP_DISPLAY_ONLY),
    # that we require a bonding (storage of the keys after pairing)
    # and that we want both secure connection and main in the middle protection.
    # Since we have CAP_DISPLAY_ONLY, we also declare a passkey that will be shown to the user
    # to be entered on the master (i.e. the smartphone) to finalize the bonding.
    ble.security(
        capabilities=ble.CAP_DISPLAY_ONLY,
        bonding=ble.AUTH_BOND,
        scheme=ble.AUTH_SC|ble.AUTH_MITM,
        key_size=16,
        passkey=225575)
    # To do so, we need a callback to display the passkey when needed
    ble.add_callback(ble.EVT_SHOW_PASSKEY,show_key_cb)

    # Setup advertising to 50ms
    ble.advertising(50)

    # Start the BLE stack
    ble.start()

    # Now start advertising
    ble.start_advertising()

except Exception as e:
    print(e)

# loop forever
while True:
    print(".")
    if random(0,100)<50 and notifications_enabled and connected:
        value = bytearray(cn.get_value())
        value[0]=0  # simple alert type
        if value[1]<255:
            value[1]=value[1]+1   # add a notification
        print("Adding a new notification, total of",value[1])
        # the remaining 14 bytes can be some text 
        value[2:10] = "Zerynth!"
        # set the new value. If ble notifications are enabled, the connected device will receive the change
        cn.set_value(value)
    sleep(5000)


