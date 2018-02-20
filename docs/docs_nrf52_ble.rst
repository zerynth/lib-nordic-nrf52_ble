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

    
