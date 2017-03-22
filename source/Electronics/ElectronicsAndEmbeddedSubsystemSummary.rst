
Introduction
============

The control of highly coupled and compliant musculoskeletal systems is
a complex task. In contrast to the well established control concepts
of stiff robots widely used in industry, control strategies and
algorithms for musculoskeletal systems are still areas of active
research. In order to experiment with advanced control algorithms, it
is desirable to have a set of simple linear controllers to control the
muscle state. This baseline system fulfils two purposes; it provides a
benchmark against which more advance control schemes can be tested
and, perhaps more importantly, it provides a control interface that
can be utilised by advanced and higher level control algorithms.

In order to run a range of linear (feedback) controllers on the
MYO-Muscles, a control environment for sensing, processing and
actuation is required. Due to the modular nature of the Myorobotic
system, a heterogeneous and distributed control infrastructure has
been devised. It allows the Myorobotics developers and users to test
and control their Myorobot through the MYO-Muscles. In the following
document, the main components of the control system are described from
both a hardware and software perspective.

Overview
========

An overview of the distributed control system is presented in :numref:`my-figure`.
The Myorobotics system consists of the PC (UBUNTU 12.04 LTS) application ‘Caliper’
including the ‘MYODE’ plug-in that communicates with the Myorobot via a USB 2.0
interface. The USBFlexRay board establishes the link between the FlexRay
bus (the main Myorobotics communication system) and the PC.

.. _my-figure:
.. figure:: img\systemOverviewECUAndCommsIntegrationSeptember2014WideVersionUSBFlexRay.png
   :align: center

   Overview of the Myorobotics distributed control infrastructure.

A Myrobotics system consists of up to 6 MYO-Ganglions, which are local
32-bit floating point electronic control units (ECU) that communicate
via the FlexRay bus. Each MYO-Ganglion can control up to four MYO-Musles
in different control modes. The communication between the MYO-Muscles
and the MYO-Ganglion is established via four dedicated 5Mbit/s SPI
connections. Up to four joints can also be connected to a MYO-Ganglion.
These joints share a common controller area network (CAN) bus and
communicate their absolute joint position at a rate of 1kHz. The same
CAN bus is also utilised to read the state of up to 12 MYO-Perceptors,
scalar sensors that can be used for various purpuses like tactile or
temperature sensing. These sensors broadcast their sensory state at a
rate of 100Hz.

:numref:`Image of Sphinx (Fig. %s) <my-figure>`

MYO-Ganglion
============

The MYO-Ganglion comprises three printed circuit boards (PCB) that are
mounted on a carrier that can be clipped onto a medium-size MYO-Bone
(:numref:`her-figure`). The centre board features the main
floating-point processor, the TMS570LS20216 from Texas Instruments
running at 140MHz. Adjacent to the processor board are the power supply
and distribution boards, respectively.

.. _your-figure:
.. figure:: img\MyoGanglionConnectivity_withlabels.png
   :align: center

   The MYO-Ganglion PCB assembly: distribution board with CAN and SPI
   connections is shown on the left, the centre board is equipped with the main DSP
   (TMS570LS20216 on bottom side , not visible), the power supply board (24V) is mounted
   on the right side of the carrier.

.. _her-figure:
.. figure:: img\GanglionOnBone.jpg
   :align: center

   The MYO-Ganglion mounted on bone before cables are attached.

Addressing
----------

A Myorobot can have up to six MYO-Ganglions sharing the FlexRay bus.
Each Ganglion has a unique address which is configured using the
DIP-switches 1 to 6. In order to enable the Ganglion, one (and only one)
of the DIP switches has to be in the ON position. All others have to be
in the OFF position. If more than one DIP switch is in the ON position
the Ganglion will not participate in the FlexRay communication.
Similarly, if none of the switches are in the ON position, the Ganglion
will not participate in the FlexRay communication. This provides a
convenient way to temporarily disable a Ganglion that is not required
(see also :numref:`mytable`).

.. _mytable:

.. table:: MYO-Ganglion addressing scheme

    +-------+-------+-------+-------+-------+-------+-----------------------+
    | SW1   | SW2   | SW3   | SW4   | SW5   | SW6   | Address / C++ index   |
    +=======+=======+=======+=======+=======+=======+=======================+
    | 1     | 0     | 0     | 0     | 0     | 0     | [0]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    | 0     | 1     | 0     | 0     | 0     | 0     | [1]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    | 0     | 0     | 1     | 0     | 0     | 0     | [2]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    | 0     | 0     | 0     | 1     | 0     | 0     | [3]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    | 0     | 0     | 0     | 0     | 1     | 0     | [4]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    | 0     | 0     | 0     | 0     | 0     | 1     | [5]                   |
    +-------+-------+-------+-------+-------+-------+-----------------------+
    |             any other combination             |invalid / not connected|
    +-------+-------+-------+-------+-------+-------+-----------------------+



USB-FlexRay Bridge
==================

In order to connect MYODE with a Myorobot, a USB-FlexRay bridge is
provided. This system is illustrated in :numref:`his-figure`. To
connnect to the PC, a mini-USB lead is necessary. The bridge board is
also supplied with 24V, which should be the same voltage source that
supplies the Myorobot to establish a common ground connection. The
connection to the Myorobot, i.e., via the MYO-Ganglions, is established
through a 2-wire FlexRay interface.

.. _his-figure:
.. figure:: img\USBFlexRayBridgeCutOut_withlabels.png
   :align: center

   The Myorobotics USB-FlexRay Bridge

FlexRay is a differential serial bus and the FlexRay cables used for a
Myorobot are a simple twisted pair wires. The two FlexRay signal lines
are referred to as **FRp** (FlexRay Plus, the positive signal) and
**FRm** (FlexRay Minus, the negative signal). The FlexRay cable provided
with your Myorobotic system is shown in :numref:`its-figure`. The
pink cable is used for the FRp and the green cable for the FRm signal.
The MYO-Ganglions feature two pairs of FlexRay connections (see Figure
[fig:MyoGanglionConnectivity]) which affords easy daisy-chaining of
multiple MYO-Ganglions.

.. _its-figure:
.. figure:: img\FlexRayCable_withlabels.png
   :align: center

   The FlexRay cable used for Myorobotic system: green is the FRm signal, pink
   the FRp signal.

Motor Driver
============

In order to drive the MYO-Muscles, a motor driver board is provided.
This is illustrated in :numref:`our-figure`. The motor driver
board is supplied with 24V and communicates with the MYO-Ganglion via a
5MHz SPI connection. It provides sockets to connect the MYO-Muscle motor
as well as a further connection for the spring-displacement sensor. For
further hardware developments and other extensions, there is also a CAN
interface and a micro-USB connection. However, they are not required
when building a Myorobot.

.. _our-figure:
.. figure:: img\MotorDriverBoardCutOut-withlabels.png
   :align: center

   The Myorobotics motor driver board.

Spring Displacement Sensor
--------------------------

To measure the displacement of the spring (a proxy for tendon force), a
spring displacement sensor is connected to the motor driver board. The
sensor is supplied via the motor driver board and connected via a 6-pin
JST connector [1]_ as depicted in :numref:`their-figure`.

.. _their-figure:
.. figure:: img\displacementSensor_withlabels.png
   :align: center

   The spring displacement sensor and connector: Please note that the connector
   cable is not symmetric. Consequently, one end of the connector cable (marked with S or D)
   is plugged into the sensor board (A) and the other end (B) (marked with M) is plugged into
   the motor driver board (:numref:`our-figure`)

Wiring Scheme: Spring Displacement Sensor - Motor Driver Board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+----------------------------------+-------+--------+--------+-----+-------+-------+
| **Signal Name**                  | GND   | EncA   | EncB   | O   | Idx   | +5V   |
+==================================+=======+========+========+=====+=======+=======+
| **Displacement Sensor, pin #**   | 1     | 2      | 3      | 4   | 5     | 6     |
+----------------------------------+-------+--------+--------+-----+-------+-------+
| **Motor Driver Board, pin #**    | 5     | 3      | 2      | 1   | 4     | 6     |
+----------------------------------+-------+--------+--------+-----+-------+-------+

.. figure:: MotorToSpringSensorConnectivity.pdf
   :alt: Cables and connectors to connect the spring displacement sensor
   with the motor driver board; red circles mark the applicable
   connectors on the printed circuit boards.
   :width: 50.0%

   Cables and connectors to connect the spring displacement sensor with
   the motor driver board; red circles mark the applicable connectors on
   the printed circuit boards.

.. _the-figure:
.. figure:: img\Cablesandconnectors.png
    :align: center

    Cables and connectors to connect the spring displacement sensor with the motor
    driver board; red circles mark the applicable connectors on the printed circuit boards.

MYO-Muscle Assembly
-------------------

To illustrate how a motor driver board is mounted on the MYO-Muscle
please refer to :numref:`a-figure`. The connector for the
spring displacement sensor should be facing the spring. Two screws are
sufficient to mount the motor driver board on the MYO-Muscle as shown in
:numref:`a-figure`.

.. _a-figure:
.. figure:: img\motordriverboard_withlabels.png
    :align: center

    Motor driver board mounted on MYO-Muscle

Connectivity
------------

The motor driver board has to be connected to the MYO-Ganglion board
using the 5-pin JST connectors [2]_ . Depending where the motor driver
board is plugged in (SPI0, SPI1, SPI2 or SPI3) the associate MYO-Muscle
can be addressed with the corresponding index in MYODE. In other words,
the address of a motor driver board (and therefore the MYO-Muscle) is
dependent upon the SPI connector it is connected to (see :numref:`atable`).

.. _atable:

.. table:: Motor driver addressing scheme

    +------------------+-----------------------+
    | SPI Connection   | Address / C++ index   |
    +==================+=======================+
    | SPI0             | [0]                   |
    +------------------+-----------------------+
    | SPI1             | [1]                   |
    +------------------+-----------------------+
    | SPI2             | [2]                   |
    +------------------+-----------------------+
    | SPI3             | [3]                   |
    +------------------+-----------------------+

Wiring Scheme SPI Connector: Ganglion Distribution Board - Motor Driver Board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+-----------------------------------------+--------+--------+-------+------+-------+
| **Signal Name**                         | SOMI   | SIMO   | Clk   | SS   | Gnd   |
+=========================================+========+========+=======+======+=======+
| **Ganglion Distribution Board, pin#**   | 1      | 2      | 3     | 4    | 5     |
+-----------------------------------------+--------+--------+-------+------+-------+
| **Motor Driver Board, pin #**           | 1      | 2      | 4     | 3    | 5     |
+-----------------------------------------+--------+--------+-------+------+-------+
.. _one-figure:
.. figure:: img\cablesandconnectors_withlabels.png
    :align: center
    Cables and connectors to connect the SPI of the distribution board with the
    motor driver board; red circles mark the applicable connectors on the printed circuit boards.

Joint Sensor Board
==================

The MYO-Joints are equiped with an absolute position sensors. An
interface board (joint sensor board) is mounted on each joint as shown
in Figure [fig:JointAndJointSensorBoard]. The joint position is sent to
the MYO-Ganglion via CAN. The joint sensor board reads the magnetic
joint sensor (within the joint) at a rate of 16kHz. A filtered value of
this joint position (moving average filter) is sent to the MYO-Ganglion
at a rate of 1kHz. Up to 4 joint sensors can be connected to the
MYO-Ganglion on a shared CAN bus. The DIP-switches on the joint sensor
board are required to configure the CAN message ID (communication
address). The DIP switches (S1, S2 and S3) are read after power-on
reset. Manipulation of the switches during operation has no effect. For
a 1DOF joint DIP switches 1 and 2 are used to set the address (0b00,
0b01, 0b10 or 0b11). Switches 3, 4 and 5 must be in the off
position [3]_. Switch 6 enables a CAN termination resistor. One (and
only one) of the joint sensor boards connected to a MYO-Ganglion must
have the termination resistor enabled (i.e. switch 6 ON). In general,
CAN requires two :math:`120\Omega` termination resistors. One of them is
present on the MYO-Ganglion board and therefore only one of the joint
sensors should have its termination resistor enabled.

Connectivity
------------

The joint sensor board is supplied with a 4-pin [4]_ JST connector and
should be directly connected to the MYO-Ganglion using any of the 5
available CAN connectors. The address of the joint is subject to the
address of the joint sensor board, using DIP switches 1 and 2 as shown
in Table [tab:JointSensorAddress].

During 1DOF operation, only one CAN message with the MsgID indicated by
switches S1 and S2 is sent. For 2DOF operation two CAN messages are
sent, the first one has the MsgID indicated by switches S1 and S2, the
second CAN message has the ID indicated with switches S1 and S2 plus 1.

+------+------+------+---------------------+
| S1   | S2   | S3   | messageIDs on bus   |
+======+======+======+=====================+
| 0    | 0    | 0    | 0x50                |
+------+------+------+---------------------+
| 0    | 0    | 1    | 0x50 and 0x51       |
+------+------+------+---------------------+
| 0    | 1    | 0    | 0x51                |
+------+------+------+---------------------+
| 0    | 1    | 1    | 0x51 and 0x52       |
+------+------+------+---------------------+
| 1    | 0    | 0    | 0x52                |
+------+------+------+---------------------+
| 1    | 0    | 1    | 0x52 and 0x53       |
+------+------+------+---------------------+
| 1    | 1    | 0    | 0x53                |
+------+------+------+---------------------+
| 1    | 1    | 1    | 0x53                |
+------+------+------+---------------------+

Table: CAN message IDs of the sensor board as a function of the DIP
Switches S1,S2 and S3. S6 (not shown in the table) is used to switch the
CAN termination on and off, S4 is for calibration and needs to be set to
off during operation. S5 is currently reserved.

+------+------+-----------------------+
| S1   | S2   | Address / C++ index   |
+======+======+=======================+
| 0    | 0    | [0]                   |
+------+------+-----------------------+
| 0    | 1    | [1]                   |
+------+------+-----------------------+
| 1    | 0    | [2]                   |
+------+------+-----------------------+
| 1    | 1    | [3]                   |
+------+------+-----------------------+

Table: Joint sensor addressing scheme for 1DOF operation. In the 2DOF
configuration two consecutive indices are valid, i.e. either 0 and 1, 1
and 2, or 2 and 3. The joint addresses have to be selected in such a
manner that never more than one joints sends a given CAN message ID.
Refer to table [tab:canMessageIDSensorBoard] for details on CAN
addresses.

Wiring Scheme: Joint Angle Sensor Board - Ganglion Distribution Board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+------------------------------------------+---------+---------+-------+-------+
| **Signal Name**                          | CAN-H   | CAN-L   | Gnd   | +5V   |
+==========================================+=========+=========+=======+=======+
| **Sensor board, pad #**                  | 1       | 2       | 3     | 4     |
+------------------------------------------+---------+---------+-------+-------+
| **Ganglion Distribution Board, pin #**   | 3       | 2       | 1     | 4     |
+------------------------------------------+---------+---------+-------+-------+

.. figure:: jointAngleSensorWithConnector-crop.pdf
   :alt: Cables and connectors to connect the joint angle sensor board
   to the ganglion distribution board; red circles mark the applicable
   connectors on the printed circuit boards.
   :width: 50.0%

   Cables and connectors to connect the joint angle sensor board to the
   ganglion distribution board; red circles mark the applicable
   connectors on the printed circuit boards.

Wiring Scheme: Analogue joint sensor - Joint Angle Sensor Board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The analogue joint sensor are soldered straight into the soldering pad
on the joint angle sensor boards. The joint angle sensor board can
output 5V or 3.3V on pins 6 and 8, depending on the components
configured onto the joint angle sensor board.

+---------------------------+-------+-------+------------+------------+-------+-------+
| **Signal Name**           | Gnd   | Gnd   | +5V/3.3V   | +5V/3.3V   | AN0   | AN1   |
+===========================+=======+=======+============+============+=======+=======+
| **Sensor board, pad #**   | 5     | 7     | 6          | 8          | 9     | 10    |
+---------------------------+-------+-------+------------+------------+-------+-------+

Wiring Scheme: 5V and 3.3V configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The joint angle sensor board can operate with 5V or 3.3V sensors,
depending on the resistors populated and solder-bridges made. Details
can be seen in Figure [fig:jointAngleSensor5VConfiguration]. Resistors
R5,R6,R7 and R8 are required to divide down the sensor output, in case
of the 5V configuration, to the 3.3 analogue input voltage range of the
micro-controller on the sensor board. One, and only one, solder-bridge
(SB) between the 5V pad or 3.3V pad and the :math:`V_{supply}` pad is
required to supply the sensor with the appropriate voltage. For 3.3V
operations resistors R7 and R8 should be removed and R5 and R6 replaced
with a 0\ :math:`\Omega` resistor.

.. figure:: boardConfiguration5Vand3_3V-crop.pdf
   :alt: PCB with components for 5V operation:
   :math:`R5=5k\Omega,R6=5k\Omega,R7=10k\Omega,R8=10k\Omega` and
   solder-bridge (SB) implementing the connection between +5V and the
   sensor supply voltage :math:`V_{sensor}`. For 3.3V operation the
   solder-bridge is required between the 3.3V pad and
   :math:`V_{senosr}`. Importantly, the SB between +5V and
   :math:`V_{sensor}` needs then be removed. In 3.3V operation R7 and R8
   should be removed and R5 and R6 replaced with a :math:`0\Omega`
   resistor (or a resistance :math:`<10\Omega`). Red tracks/pads mark
   the PCB top, blue tracks/pads are on the bottom side of the PCB.
   :width: 50.0%

   PCB with components for 5V operation:
   :math:`R5=5k\Omega,R6=5k\Omega,R7=10k\Omega,R8=10k\Omega` and
   solder-bridge (SB) implementing the connection between +5V and the
   sensor supply voltage :math:`V_{sensor}`. For 3.3V operation the
   solder-bridge is required between the 3.3V pad and
   :math:`V_{senosr}`. Importantly, the SB between +5V and
   :math:`V_{sensor}` needs then be removed. In 3.3V operation R7 and R8
   should be removed and R5 and R6 replaced with a :math:`0\Omega`
   resistor (or a resistance :math:`<10\Omega`). Red tracks/pads mark
   the PCB top, blue tracks/pads are on the bottom side of the PCB.

Calibration Procedure
---------------------

The joints should be calibrated before the first operation. This makes
sure that the digital outputs of the sensor board map symmetrically to
the physical range of the analogue sensors. A calibrated sensor will
broadcast a value of :math:`2048_{dec}` in the centre position and a
value between 0 and :math:`2048_{dec}` at the physical negative end-stop
(depending on range). The value at the positive end-stop will be between
:math:`2048_{dec}` and :math:`4095_{dec}`, again depending on the
physical range. The calibration only needs to be performed once when
connecting the sensor board to the physical joint and sensor, the
calibration data is stored permanently in the flash memory of the joint
angle sensor board. However, the procedure can be repeated if mistakes
were made during calibration or if he sensor board is mounted onto
another joint. The calibration data is agnostic to the to the joint
address in principle. However, it is easiest to perform the calibration
when joint ID zero (S0=0, S1=2) is selected. The calibration works for
1DOF and 2DOF operation. The following procedure will lead to a
successful calibration:

-  S0 and S1 are set to 0 (off),S4 is off, S3 off in 1DOF operation or
   S3 on for 2DOF operation

-  power up joint angle sensor board

-  set S4 to on

-  move joint to negative position, hold there

-  flick S0 on and off again

-  move joint to positive position, hold there

-  flick S1 on and off again

-  set S4 to off

-  calibration has been performed

Controllers and Software Interface
==================================

The MYO-Ganglion implements the linear-feedback controllers for the
MYO-Muscles. Currently, five control modes are possible: *raw, position,
velocity, force* and *torque*. In the raw mode, no feedback controller
is enabled. Rather, the muscle is driven in an open-loop mode where the
motor supply voltage can be varied between :math:`\pm 100\%`. The
remaining four control modes use the freely configurable linear-feedback
control topology depicted in Figure [fig:LFCUpdated].

To be clear, these controllers run on the MYO-Ganglion autonomously.
They are configured via MYODE (control parameters, cycle time, etc)
during the start-up phase of the user’s high-level controller running
within MYODE. By default, the gains are all set to zero, so no control
action is issued. During run-time, MYODE sends the reference values to
the controllers which can happen at any point in time and with arbitrary
update rates. Furthermore, the control parameters can also be changed
during runtime. Note, however, that the control parameters are not
stored on the MYO-Ganglion. Following reset, all the controllers need to
be re-configured.

Configuring a Controller
------------------------

The MYODE interface to a muscle provides a controller configuration
method, namely ``void setControllerParams(const comsControllerMode controlMode, control_Parameters_t controlParameters)``.
The control mode is implemented as an enumeration the valid modes of
which are:
``Raw, Torque, Velocity, Position, Force``.
To set the control parameters an instance of structure
``control_Parameters_t`` needs to be created, filled and then passed
to the ``setControllerParams(.)`` method. The structures required are
shown (including comments) in Figure
[fig:controllerParametersStructure]. In addition to the values found
in the controller diagram in Figure [fig:LFCUpdated], the structure
also provide entries for the controller update frequency
(``float32 timePeriod``) in :math:`\mu s`, and values to map the
physical system parameters into appropriate units (e.g.
``float32 radPerEncoderCount, float32 torqueConstant``).

The four parameter array ``float32 polyPar[4]`` describes the
non-linear mapping of the spring displacement measurement to a force.

Communication Timing
--------------------

Before a snippet of example code is presented, let us briefly consider
the timing behaviour of this (partly) asynchronous communication
system. In principle, four different timing cycles can be
distinguished and they are illustrated in Figure
[fig:CommunicationCycleTimes]. At the highest level is the **user
application (UA)** running as part of MYODE. Typically, the cycle time
of this control loop is in the tens of milliseconds range (e.g.
:math:`20ms`) and is set by the user. Since a standard Ubuntu
installation is used, it is important to note that the cycle time of
the UA is not ‘hard real-time’ and some variance on the timing is to
be expected. In the UA, data from the Myorobot is read, such as motor
velocity or joint angles, or set in the case of tendon force and motor
position. All those operations are thread-safe.

Data is exchanged with the Myorobot via a thread that is hidden from
the user and referred to as the **USB interface (USBI)**. The USBI
also runs as a ‘soft real-time’ system with a nominal update rate of
500Hz. In other words, data exchange between the UA (via the USBI) and
the Myorobot is also limited to a minimum update rate of 2ms.

The next level of communication is realised with the **USB-FlexRay
bridge (UFR)** (see section [sec:USB-FlexRay]). Here, the USB data is
exchanged with the ‘hard real-time’ FlexRay bus that forms the
communication backbone of the Myorobot, allowing the exchange of data
between the UFR and the MYO-Ganglions in a fully synchronous and
time-trigger fashion at a rate of 1kHz.

The lowest level in this communication chain is formed by the
**linear-feedback controllers (LFC)**\ (see Figure [fig:LFCUpdated])
running on the MYO-Ganglions. The controllers run in a ‘hard real-time’
loop on the MYO-Ganglion and exchange data with the FlexRay bus and the
motor driver boards (**MD**); see section [sec:MotorDriver]. As
explained above, the cycle time of the linear-feedback controllers is
user configurable by setting the ``float32 timePeriod`` variable of the
structure `` control_Parameters_t ``\ and configuration of the
controller via the ``setControllerParams(.)`` method. The minimum cycle
time is :math:`400\mu s`.

An Example
----------

| To further illustrate the control of a Myorobot using MYODE, a minimal
  example is shown in Figure [fig:GeneralControlLoopExample]. The
  ``GeneralControlLoop`` class is derived from the
  ``IGeneralControlLoop`` interface class and receives a pointer to the
  ``IRobot`` class (``p_robot``) in its constructor. This establishes
  the link to the physical or simulated robot. The
  ``IGeneralControlLoop::init()`` method is a pure virtual function and
  needs to implemented by the user. It is called once after the
  controller has been instantiated. The member variable
  ``localParameters`` is a private instance of the control parameter
  structure ``control_parameters_t`` and is filled with the motor
  control parameters. A reference to this structure is then passed to
  the ``setControllerParams(.)`` method of the MYO-Muscle. In this
  example, we configure MYO-Muscle 0 on MYO-Ganglion 0 of the Myorobot
  as a position controller
| (``p_robot->getGanglion(0)- >getMuscles()[0]->setControllerParams(Position,localParameters)``).

| The cyclic control loop, which could run at a user configurable rate
  (e.g. 20ms), is implemented with the ``IGeneralControlLoop::cycle()``
  method. As above, this pure virtual function needs to be implemented
  by the user.
| Before the actual controllers can be used, the application needs to
  check if the configuration of the controller has been completed. This
  is done by checking the
| `` p_robot->controlparameterRequestQueueEmpty()`` method. The reason
  for this check is that the configuration parameters to all controllers
  are transmitted on the FlexRay bus using a shared (dynamic) slot.
  Consequently, the configuration of several controllers will take some
  time. This is in contrast to setting reference value or
  enabling/disabling a controller, here each MYO-Ganglion can be
  addressed separately using dedicated slots on the FlexRay bus. This
  maintains the real time performance of the controllers within the
  limits outlined in section [sec:controllerConfiguration].
| When the configuration queue is empty, the controllers can be enabled
  by calling
| ``p_robot->getGanglion(0)->getMuscles()[0]->enableController().``
| The controller reference values can be set with
| ``p_robot->getGanglion(0)->getMuscles()[0]->setControllerRef(Position,referencePosition) ``.

Summary
=======

This document provides a brief introduction to the Myorobotics
electronics and embedded system in the style of a ‘quick-start guide’.
It should supply the user of a Myorobot with sufficient information to
understand the infrastructure, connectivity, software interfaces and
capabilities as well as an appreciation of the limitations of the
system. For further details, the circuit diagrams as well as the
software (documentation) on the Myorobotics repository should be
consulted.

.. [1]
   The 6-way JST SH series connectors are available from Farnell
   Components, Farnell-number 1679112; connecting wires with pre-crimped
   connectors are available via RS components (300mm RS-number 311-6675,
   150mm RS-number 311-6653).

.. [2]
   The 5-way JST SH series connectors are available from Farnell
   Components, Farnell-number 169111; connecting wires with pre-crimped
   connectors are available via RS components (300mm RS-number 311-6675,
   150mm RS-number 311-6653).

.. [3]
   Switches 3,4 are required to choose between 1DOF and 2DOF operation
   (S3) or to calibrate the joint (S4). Switch 5 is reserved

.. [4]
   The 4-way JST SH series connectors are available from Farnell
   Components, Farnell-number 1679110; connecting wires with pre-crimped
   connectors are available via RS components (300mm RS-number 311-6675,
   150mm RS-number 311-6653).
