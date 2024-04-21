.. _troubleshoot-installation:

Troubleshooting
================

This is a selection of typical problems. If your problem is not listed below, please check the issues for all related repositories:

- `myo_blink <https://github.com/Roboy/myo_blink/issues?utf8=%E2%9C%93&q=is%3Aissue>`_
- `flexrayusbinterface <https://github.com/Roboy/flexrayusbinterface/issues?utf8=%E2%9C%93&q=is%3Aissue%20>`_
- `MyoRobotics-Documentation <https://github.com/MyoRobotics/MyoRobotics-Documentation/issues?utf8=%E2%9C%93&q=is%3Aissue%20>`_



----

.. _device-not-opened:

Could not connect to the myo motor: device not opened
-------------------------------------------------------

There are a number of possible reasons:

You do not have read/write access to the device /dev/ttyUSBn. Where **n** is a number
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Verify this by running:

.. code-block:: bash

  cat /dev/ttyUSBn

- If you get a 'device not found' you didn't replace the right number for n in ttyUSBn
- If you do not get anything, just a clean return you have correct access rights. Press Ctrl+C and find the problem elsewhere.
- If you get a 'permission denied' then you have verified this problem. Add yourself to the 'dialout' group as described here: :ref:`add-to-dialout`.

----

The configured serial number does not match the serial number of the USB2FlexRay adapter
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Go and **double check** it by following the instructions here: :ref:`find-set-usb-serial`

----

You did not install the dependencies of flexrayusbinterface
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Go and do it. No harm in doing it twice.: :ref:`install-flexrayusbinterface`

----

You did not install the udev rules
++++++++++++++++++++++++++++++++++++
- Check if there is a file called '30-ftdi.rules' in '/etc/udev/rules.d/':

  .. code-block:: bash

    ls -alh /etc/udev/rules.d
    # if you don't get back anything try it with sudo:
    # sudo ls -alh /etc/udev/rules.d

- The file should contain a line with something like this at the end:

  .. code-block:: bash

    /sys/bus/usb/drivers/ftdi_sio/unbind

- If it does, compare it with the file supplied by 'flexrayusbinterface'. It is located at:

  .. code-block:: bash

    roscd flexrayusbinterface/udev

- If they match, there might be something wrong with these rules for your system. Try the option below.

----

Something in the udev rules is not right
++++++++++++++++++++++++++++++++++++++++++
- Manually try to unload the ftdi_sio and usbserial kernel modules as described in the :ref:`Manually unloading ftdi_sio and usbserial <install-flexrayusbinterface>` and try the roslaunch command again
- If that does help, use the manually loading as a workaround. Research udev files and how do unload kernel modules. If you are stuck or found a better solution, please create an issue on github at: `flexrayusbinterface <https://github.com/Roboy/flexrayusbinterface/issues?utf8=%E2%9C%93&q=is%3Aissue%20>`_
- If that does not help find the problem somewhere else. Have you checked your permissions as in the first step?
