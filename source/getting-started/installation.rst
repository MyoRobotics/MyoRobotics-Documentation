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

Run it
********
In different terminals run:


.. code-block:: console

   source .../MyoArm/devel/setup.bash
   roscore
   rostopic echo -c /rrbot/joint_states
   rostopic echo -c /rrbot/joint_effort/controller/command
   roslaunch ros_control_boilerplate rrbot_hardware.launch
   rosrun ros_control_boilerplate keyboard_teleop


.. _myo_blink: https://github.com/Roboy/myo_blink
.. _ros_control_boilerplate fork: https://github.com/compiaffe/ros_control_boilerplate/tree/MyoArm
.. _flexrayusbinterface: https://github.com/Roboy/flexrayusbinterface/tree/develop
.. _ROS kinetic: http://wiki.ros.org/kinetic/Installation
