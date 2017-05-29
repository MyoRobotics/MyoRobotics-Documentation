.. _make-your-own:

Installation
===============
How to get started with a set of MyoMuscles, Ganglia and Flexray2USBAdapter.

Cabling and Power
------------------
.. WARNING::
  Check if:

  * all motor cables are plugged in correctly. There are 2 ribbon cables (grey and rainbow coloured) that connect each motor with its motor driver board
  * all :ref:`sensor displacement cables <eaess-sds-cs>` are plugged in correctly
  * all motor driver boards are connected to power with common ground and power is turned off
  * all motor driver boards are connected to their respective ganglion (via SPI)
  * all joint sensors are connected to ganglia
  * all ganglia are connected to power with common ground and power is turned off
  * all ganglia are connected to the flexray bus
  * all power supplies are set to output at the very most 24V (for basic testing the 13 muscle MyoArm will run off a single 3A power supply)
  * the Flexray2USB adapter is connected to power with common ground and power is turned off
  * the Flexray2USB adapter is connected to the flexray bus
  * turn the power on and check if

    * all driver boards are lighting/flashing green
    * the joint has a blinking light whose frequency changes with the joint's angular displacement

  * the software starts

----

Software Installation
----------------------
To get started we recommend installing both the `myo_blink`_ and `ros_control_boilerplate fork`_ examples on top of the required `flexrayusbinterface`_.

These examples require `ROS kinetic`_ to be installed on Ubuntu 16.04.

----

.. _add-to-dialout:

Allow your user to access the serial device
********************************************

By default Ubuntu does not let you access your serial adapters. To change that we will add your user to the group 'dialout'.
Execute the following in a terminal:

.. code-block:: bash

  sudo usermod -a -G dialout $(whoami)

This only needs to be done once.

----

Create a new workspace
************************
Create a new folder that will contain your ROS / catkin workspace and all code.

.. code-block:: bash

   source /opt/ros/kinetic/setup.bash
   mkdir -p ~/MyoArm_ws/src && cd MyoArm_ws/src && catkin_init_workspace
   cd ..
   catkin_make

Now add the workspace to your ~/.bashrc so that it gets automatically sourced upon opening a shell:

.. code-block:: bash

   echo 'source ~/MyoArm_ws/devel/setup.bash' >> ~/.bashrc

----

.. _install-flexrayusbinterface:

Install flexrayusbinterface
****************************

.. code-block:: bash

   roscd && cd ../src
   git clone https://github.com/roboy/flexrayusbinterface.git -b develop
   roscd flexrayusbinterface && ./install_deps.sh

.. IMPORTANT:: The ftd2xx driver does not get loaded automatically. In order to use it you need to either install our udev rules [#fudev]_ (recommended):

  .. code-block:: bash

    roscd flexrayusbinterface && sudo ./install_udev_rules.sh


  Or manually unload the standard drivers **every time you re-plug** the Flexray2USBInterface board:

  .. _manual-unload-kernel-modules:

  .. code-block:: bash

    sudo rmmod ftdi_sio
    sudo rmmod usbserial

----

Install myo_blink
*****************
Clone
+++++++

.. code-block:: bash

   roscd && cd ../src
   git clone https://github.com/roboy/myo_blink.git -b master


.. _find-set-usb-serial:

Configure the myo_blink software example.
+++++++++++++++++++++++++++++++++++++++++

All system configuration is placed inside a yaml file in the 'config' directory of this package.
Most importantly it **contains the serial number** of the USB2Flexray adapter. Adjust it to your devices ID.

.. HINT::

  1. **Find your device mounting location in /dev**

    All unix systems treat everything (including devices) as files. So first we want to find where your Ubuntu has mounted the USB2Flexray adapter.

    Unplug the USB cable of the USB2Flexray adapter and **in a terminal do one by one:**

    .. code-block:: bash

      ls -1 /dev > ~/before.txt

      # Plug the UBS cable back in

      ls -1 /dev > ~/after.txt

      diff ~/before.txt ~/after.txt

    You should see a few lines, one of which should start with:

    .. code-block:: bash

      > ttyUSBn

    Where 'n' is a number: This is the device location.

    ttyUSBn is the name of your USB device (i. e. ttyUSB1). It has been mounted at **/dev/** as **/dev/ttyUSBn** now let's:

  2. **Find the device's serial number**

    Use the following command, but replace the **ttyUSBn** with the above found name starting with **ttyUSB**:

    .. code-block:: bash

      /bin/udevadm info --name=/dev/ttyUSBn | grep SERIAL_SHORT

    The returned string is the unique serial of the USB2FLEXRAY adapter, please copy it.

  3. **Update the .yaml file**

    Replace the string after the tag **serial:** in the yaml file located in the **config** directory of the myo_blink package with the newly found serial.

----

Install ros_control_boilerplate fork
************************************

.. code-block:: bash

   roscd && cd ../src
   git clone https://github.com/compiaffe/ros_control_boilerplate.git -b MyoArm

.. IMPORTANT::

  Also set the serial number in the corresponding yaml file as per :ref:`find-set-usb-serial`.
  The yaml file is placed in

  .. code-block:: bash

    roscd ros_control_boilerplate/rrbot_control/config

----


Install all ROS dependencies
****************************

.. code-block:: bash

   apt-get install -y ros-kinetic-rosparam-shortcuts ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-control-msgs ros-kinetic-urdf ros-kinetic-control-toolbox ros-kinetic-robot-state-publisher libgflags-dev libncurses5-dev libncursesw5-dev wget vim

----

Build it
***************

.. code-block:: bash

   roscd && cd ..
   catkin_make


----

Run it using the myo_blink example application
***********************************************

For using ROS effectively, you will need a large number of terminals open at the same time. I recommend using the terminal app: **terminator**.
Here you can split the screen into multiple terminals or add tabs. Once it is installed, see what a *right-click* allows you to do.

**Install it using:**

.. code-block:: bash

  sudo apt-get install terminator -y

----

In different (terminator) terminals run:

.. code-block:: bash

   source .../MyoArm/devel/setup.bash

Then **one** of the following:

.. code-block:: bash

  roscore
  roslaunch myo_blink myo_blink.launch
  rostopic list

For the last one you should now see a list of 13 topics starting with */myo_blink/muscles/*

----

To see the state of a muscle you need to subscribe to its topic: Every muscle has a topic where it publishes it's state. These are the 13 topics found above.

i.e. listen to the topic of the *biceps* muscle as follows:

.. code-block:: bash

  rostopic echo -c /myo_blink/muscles/biceps

.. IMPORTANT:: Please note, that nothing will be published on these topics before you have sent any command to the motor!

----

In order to control a motor you need to send a rosservice call to it **in a new console**:

.. code-block:: bash

   rosservice call /myo_blink/move "biceps
   action: 'move with'
   setpoint: 0.0"

.. IMPORTANT:: When typing the rosservice call parameters **autocomplete is your friend**: Start by typing *rosservice call /myo_blink/move* and then press *tab* once or twice. ROS will autocomplete your text as good as it can. All you still need to do is fill in the action, to one of the options shown below and type in a setpoint.

**Control mode (action):**

* 'move to' - PositionController
* 'move with' - VelocityController
* 'keep' - Effort / ForceController


.. _myo_blink: https://github.com/Roboy/myo_blink
.. _ros_control_boilerplate fork: https://github.com/compiaffe/ros_control_boilerplate/tree/MyoArm
.. _flexrayusbinterface: https://github.com/Roboy/flexrayusbinterface/tree/develop
.. _ROS kinetic: http://wiki.ros.org/kinetic/Installation

.. [#fudev] The udev rules are based on this article: https://www.ikalogic.com/ftdi-d2xx-linux-overcoming-big-problem/
