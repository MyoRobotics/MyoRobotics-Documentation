.. MyoRobotics Documentation master file, created by
   sphinx-quickstart on Tue Feb 14 08:36:09 2017.
   You can adapt this file to your liking, but it should at least
   contain the root `toctree` directive.

MyoRobotics Documentation
=====================================================
This document describes the use of the MyoRobotics toolkit,
including assembly of muscle units, joints and bones, as well as
configuration and control of through the Flexray interface.

This documentation is compiled from documentation produced by the concluded `FP7 ICT MyoRobotics Project`_, the ongoing `Roboy Project at TU Munich`_ , `Embedded Robotic Systems`_ and the `Fraunhofer IPA`_.

License
-------
It has never been decided to assign a license to this work.

The code is owned by TU Munich.

The CAD designs are owned by Fraunhofer IPA.

.. _FP7 ICT MyoRobotics Project (dead): http://myorobotics.eu/the-myo-project/
.. _Roboy Project at TU Munich: http://roboy.org
.. _Embedded Robotic Systems (dead): http://embedded-robotic-systems.co.uk/
.. _Fraunhofer IPA: http://www.ipa.fraunhofer.de/baukasten_fuer_softrobotik.html




The main documentation for the site is organized into a couple sections:

* :ref:`getting-started`
* :ref:`make-your-own`
* :ref:`suppliers_partners`


.. _getting-started:

.. toctree::
   :maxdepth: 2
   :glob:
   :caption: Using MyoRobotics

   getting-started/*



.. _designPrimitives:

.. toctree::
   :maxdepth: 2
   :glob:
   :numbered:
   :caption: Myo Toolbox

   designPrimitives/overview.rst
   designPrimitives/designPrimitivesLibrary.rst
   designPrimitives/accessories.rst
   designPrimitives/overview-bones.rst
   designPrimitives/muscle.rst
   designPrimitives/joints.rst
   designPrimitives/jointSensor.rst
   designPrimitives/motorDriver.rst
   designPrimitives/ganglion.rst
   designPrimitives/usbFlexRayAdapter.rst
   designPrimitives/alternativeDesigns.rst



.. _make-your-own-robot:

.. toctree::
   :maxdepth: 2
   :glob:
   :numbered:
   :caption: Production & Assembly

   assembly/overview.rst
   assembly/overview-bones*
   assembly/overview-joints*
   assembly/overview-muscles*
   assembly/overview*




.. _controllers:

.. toctree::
    :maxdepth: 2
    :glob:
    :numbered:
    :caption: Control, Software

    controllers/FirmwareInterface.rst
    controllers/D4.1_update.rst



.. _suppliers_partners:

.. toctree::
   :maxdepth: 2
   :glob:
   :caption: Suppliers and Partners

   partners/*
