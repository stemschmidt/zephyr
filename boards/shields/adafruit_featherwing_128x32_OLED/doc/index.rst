.. _adafruit_featherwing_128x32_OLED:

Adafruit FeatherWing 128 x 32 OLED
##################################

Overview
********

The Adafruit FeatherWing with a
resolution of 128x32 pixels, is based on the SSD1306 controller.
More information about the shield can be found
at the `Adafruit website`_.

Pins Assignment of the Adafruit 
========================================================

+-----------------------+---------------------------------------------+
| Shield Connector Pin  | Function                                    |
+=======================+=============================================+
+-----------------------+---------------------------------------------+
| SDA                   | |
+-----------------------+---------------------------------------------+
| SCL                   | |
+-----------------------+---------------------------------------------+

.. note::

Requirements
************

This shield can only be used with a board which provides a configuration
GPIO interfaces (see :ref:`shields` for more details).

Programming
***********

Set ``--shield adafruit_featherwing_128x32_OLED`` when you invoke ``west build``. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/display/lvgl
   :board: adafruit_feather_nrf52840
   :shield: adafruit_featherwing_128x32_OLED
   :goals: build

If the shield is connected to a board which has Arduino Nano connector,
set ``--shield adafruit_featherwing_128x32_OLED`` when you invoke ``west build``.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/display/lvgl
   :board: arduino_nano_33_ble
   :shield: adafruit_featherwing_128x32_OLED
   :goals: build

References
**********

.. target-notes::

.. _Adafruit OLED FeatherWing (128x32) website:
   https://learn.adafruit.com/adafruit-oled-featherwing
