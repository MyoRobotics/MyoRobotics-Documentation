Installation
===============
How to get started with a set of MyoMuscles, Ganglia and Flexray2USBAdapter

Cabling and Power
------------------


Software Installation
----------------------
To get started we recommend installing both the `myo_blink`_ and `ros_control_boilerplate fork`_ examples on top of the required `flexrayusbinterface`_.

These examples require `ROS kinetic`_ to be installed on Ubuntu 16.04.

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

Install flexrayusbinterface
****************************

.. code-block:: bash
   roscd && cd ../src
   git clone https://github.com/Victor-Savu/flexrayusbinterface.git -b reflex
   cd flexrayusbinterface && ./install_deps.sh

Install myo_blink
*****************

.. code-block:: bash
   roscd && cd ../src
   git clone https://github.com/Victor-Savu/myo_blink.git -b reflex

Install ros_control_boilerplate fork
************************************

.. code-block:: bash
   roscd && cd ../src
   git clone https://github.com/compiaffe/ros_control_boilerplate.git -b MyoArm

Build it
***************

.. code-block:: bash
   roscd && cd ..
   catkin_make


.. _myo_blink: https://github.com/Victor-Savu/myo_blink/tree/reflex
.. _ros_control_boilerplate fork: https://github.com/compiaffe/ros_control_boilerplate/tree/MyoArm
.. _flexrayusbinterface: https://github.com/Victor-Savu/flexrayusbinterface/tree/reflex
.. _ROS kinetic: http://wiki.ros.org/kinetic/Installation
