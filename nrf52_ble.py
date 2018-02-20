"""
.. module:: nrf52_ble

*********
NRF52 BLE
*********



This module implements the NRF52 ble driver for peripheral roles. 
It can be used for every kind of board based on `NRF52832 <https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832>`_ and has been tested on `RedBear boards <https://www.kickstarter.com/projects/redbearinc/bluetooth-5-ready-ble-module-nano-2-and-blend-2>`_

The driver is based on NRF52 SDK and can work on Virtual Machines compiled with support for Nordic SoftDevices (S132 in particular).

To use the module expand on the following example: ::

    from nordic.nrf52_ble import nrf52_ble as bledrv
    from wireless import ble
    
    bledrv.init()
    
    ble.gap("Zerynth")
    ble.start()

    while True:
        sleep(1000)
        # do things here

.. note:: This version of the driver does not support scanning and custom advertising payload

    """


@native_c("_ble_init",["csrc/ble_ifc.c"],[])
def _sd_init():
    pass


def init():
    __builtins__.__default_net["ble"] = __module__
    _sd_init()


@native_c("_ble_start",["csrc/ble_ifc.c","csrc/ble_service.c"],[])
def _start():
    pass


@native_c("_ble_get_event",["csrc/ble_ifc.c","csrc/ble_service.c"],[])
def _get_event():
    pass



def _event_loop(services,callbacks):
    try:
        while True:
            #print("Waiting event")
            evt_type, service, char, status = _get_event()
            if evt_type == 0: # GAP EVENT
                if status in callbacks and callbacks[status]:
                    try:
                        callbacks[status](None)
                    except:
                        pass
            elif evt_type == 1: # GATT EVENT
                if service in services:
                    srv = services[service]
                    if char in srv.chs:
                        ch = srv.chs[char]
                        if ch.fn is not None:
                            if status==8: # ble.WRITE
                                value = get_value(service,char)
                                val =ch.convert(value)
                            else:
                                val=None
                            try:
                                ch.fn(status,val)
                            except:
                                pass
    except Exception as e:
        pass

def start(services,callbacks):
    _start()
    thread(_event_loop,services,callbacks)
    


@native_c("_ble_set_gap",["csrc/ble_ifc.c"],[])
def gap(name,security,level,appearance,min_conn,max_conn,latency,conn_sup):
    pass


@native_c("_ble_set_services",["csrc/ble_ifc.c"],[])
def services(servlist):
    pass


@native_c("_ble_set_advertising",["csrc/ble_ifc.c"],[])
def advertising(interval,timeout,payload,list_uuids):
    pass


@native_c("_ble_start_advertising",["csrc/ble_ifc.c"],[])
def start_advertising():
    pass


@native_c("_ble_stop_advertising",["csrc/ble_ifc.c"],[])
def stop_advertising():
    pass


@native_c("_ble_start_scanning",["csrc/ble_ifc.c"],[])
def start_scanning():
    pass


@native_c("_ble_stop_scanning",["csrc/ble_ifc.c"],[])
def stop_scanning():
    pass


@native_c("_ble_set_value",["csrc/ble_ifc.c"],[])
def set_value(service, char, value):
    pass

@native_c("_ble_get_value",["csrc/ble_ifc.c"],[])
def get_value(service,char):
    pass
