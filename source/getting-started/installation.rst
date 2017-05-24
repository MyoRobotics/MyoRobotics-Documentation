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

Software Installation
----------------------
To get started we recommend installing both the `myo_blink`_ and `ros_control_boilerplate fork`_ examples on top of the required `flexrayusbinterface`_.

These examples require `ROS kinetic`_ to be installed on Ubuntu 16.04.

Create a new workspace
************************
Create a new folder that will contain your ROS / catkin workspace and all code.

.. code-block:: console

   source /opt/ros/kinetic/setup.bash
   mkdir -p ~/MyoArm_ws/src && cd MyoArm_ws/src && catkin_init_workspace
   cd ..
   catkin_make

Now add the workspace to your ~/.bashrc so that it gets automatically sourced upon opening a shell:

.. code-block:: bash

   echo 'source ~/MyoArm_ws/devel/setup.bash' >> ~/.bashrc

Install flexrayusbinterface
****************************

.. code-block:: console

   roscd && cd ../src
   git clone https://github.com/roboy/flexrayusbinterface.git -b develop
   cd flexrayusbinterface && ./install_deps.sh

.. IMPORTANT:: The ftd2xx driver does not get loaded automatically. In order to use it you need to either install our udev rules [#fudev]_ (recommended):

  .. code-block:: console

    cd flexrayusbinterface && ./install_udev_rules.sh


  Or manually unload the standard drivers **every time you re-plug** the Flexray2USBInterface board:

  .. code-block:: console

    sudo rmmod ftdi_sio
    sudo rmmod usbserial

Install myo_blink
*****************

.. code-block:: console

   roscd && cd ../src
   git clone https://github.com/roboy/myo_blink.git -b master

Install ros_control_boilerplate fork
************************************

.. code-block:: console

   roscd && cd ../src
   git clone https://github.com/compiaffe/ros_control_boilerplate.git -b MyoArm

Install all ROS dependencies
****************************

.. code-block:: console

   apt-get install -y ros-kinetic-rosparam-shortcuts ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-control-msgs ros-kinetic-urdf ros-kinetic-control-toolbox ros-kinetic-robot-state-publisher libgflags-dev libncurses5-dev libncursesw5-dev wget vim


Build it
***************

.. code-block:: console

   roscd && cd ..
   catkin_make

Run it using the myo_blink example application
***********************************************

For using ROS effectively, you will need a large number of terminals open at the same time. I recommend using the terminal app: **terminator**.
Here you can split the screen into multiple terminals or add tabs. Once it is installed, see what a *right-click* allows you to do.

**Install it using:**

.. code-block:: console

  sudo apt-get install terminator -y

----

In different (terminator) terminals run:

.. code-block:: console

   source .../MyoArm/devel/setup.bash

Then **one** of the following:

.. code-block:: console

  roscore
  roslaunch myo_blink myo_blink.launch
  rostopic list

For the last one you should now see a list of 13 topics starting with */myo_blink/muscles/*

----

To see the state of a muscle you need to subscribe to its topic: Every muscle has a topic where it publishes it's state. These are the 13 topics found above.

i.e. listen to the topic of the *biceps* muscle as follows:

.. code-block:: console

  rostopic echo -c /myo_blink/muscles/biceps

.. IMPORTANT:: Please note, that nothing will be published on these topics before you have sent any command to the motor!

----

In order to control a motor you need to send a rosservice call to it **in a new console**:

.. code-block:: console

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
