.. CommonRoad_io documentation master file, created by
   sphinx-quickstart on Tue Jul 10 09:17:31 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.


CommonRoad Carla Interface
==========================

This repository contains only the current draft version of the interface itself.
The code for converting a map from CommonRoad to OpenDRIVE is located in the CommonRoad Scenario Designer.

The software is written in Python 3.7 and tested on Linux. The usage of the Anaconda_ Python distribution is strongly recommended.

.. _Anaconda: http://www.anaconda.com/download/#download

Requirements
============

The required dependencies for running CommonRoad_io are:

* Anaconda environment
* pygame
* imageio
* (Optional) commonroad-motion-planning-library


Installation
============

Carla Interface can be installed with:

1. Create a Conda environment with Python3.7 and pip:

   $ conda create -n cr37 python=3.7 pip

2.  Activate your Conda environment:

   $ conda activate cr37

3. Install all the requirements mentioned above

   $pip install pygame imageio
   
   to install requirement open terminal in CommonRoad-CARLA Interface and run:
   
   $ pip install -e .
    


Getting Started
===============

A tutorial on the main functionalities of the project is :ref:`available here<getting_started>`.


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   user/index.rst
   api/index.rst

Contact information
===================

:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_
