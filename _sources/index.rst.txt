**Bioscara**
============
This is the documentation for Bioscara DIY SCARA robot developed at the **DTU Arena for Life Science Automation (DALSA)**.
See the table of contents below to see the available user guides. 

- The :doc:`getting_started/index` guides help you to install dependencies, build the ROS2 workspace, setup the network and finally operate the robot.
- The :doc:`further_guides/index` guides are targeted towards developers and contain other useful instructions, for example how to build this documentation or flash the joint firmware.
- The :doc:`hardware/index` guides describe the hardware assembly instructions. **Note:** These require a consolidation!
- The :doc:`code/index` links to the C++ API documentation of the ROS2 packages.


The PDF version of this documentation can be found here: :download:`bioscara.pdf <sphinx/latex/latex/bioscara.pdf>`

Architechture
-------------
The control system architecture is schematically displayed in the figure below. The yellow colors whos custom C++ hardware specific code which is described in the :doc:`code/api`. The orange colors are the ROS2 applications for motion control and trajectory generation. The red colors are SiLA applications, not yet implemented as indicated by the dashed lines.

.. image:: /assets/architecture_standalone.png
   :align: center
   :width: 75%

  
\

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   getting_started/index.rst
   further_guides/index.rst
   hardware/index.rst
   code/index.rst
..
   