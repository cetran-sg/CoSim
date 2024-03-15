# CoSIM Documentation

This website contains comprehensive documentation for the CoSIM framework.

![Carla screenshot](introduction_screenshot.jpg)
*A screenshot from Carla running as part of the CoSIM framework*

## Introduction

CoSIM stands for CETRAN open-source SIMulation framework. The framework is modular with different tools performing distinct tasks and the framework is able to achieve full system-level software-in-the-loop simulation testing of an autonomous vehicle (AV).

The co-simulation architecture incorporates the following elements:

1. A highly customizable and modular simulation and scenario framework built on top of the open-source [Carla simulator](https://github.com/carla-simulator/carla).
2. A reference traffic simulation implementation using the open-source [SUMO microscopic traffic simulator](https://github.com/eclipse-sumo/sumo) co-simulated with Carla.
3. A reference implementation of high-fidelity sensor co-simulation from [Ansys AVXcelerate](https://www.ansys.com/products/av-simulation/ansys-avxcelerate-sensors). A valid AVXcelerate license is required to use this functionality.
4. A reference ADS bridge to [Baidu Apollo](https://github.com/ApolloAuto/apollo) from the Carla simulation framework. The bridge is highly parallelized with multiprocessing-based Carla clients and communicates over a custom low overhead TCP messaging protocol developed by CETRAN.
5. A map creation workflow with [Mathworks RoadRunner](https://www.mathworks.com/products/roadrunner.html) to build accurate maps for each of the tools in the simulation toolchain.
6. And finally, various custom-developed tools and scripts used to facilitate interconnection between disparate aspects of the simulation framework.

This simulation framework and its reference implementation, as described here, consist of open-source tools and components. However, the same general architecture can be used and supplanted with any commercial or custom/proprietary tools as needed.

The framework is able to output sensor data and incorporate realistic and randomized road traffic and pedestrian interactions along with the capability to script complex scenarios which can be used to challenge the various subsystems of an AV.

## Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.
    src/
        ads/
            ApolloBridgeServer.py           # The main server-side script for the Apollo bridge. Run inside Apollo Docker.
            cyberReader.py                  # Module to read control commands from Apollo CyberRT.
            cyberWriter.py                  # Module to write simulation messages into Apollo CyberRT.
        sim/
            main.py                         # The main script for the CoSIM simulation framework.
            ApolloBridgeClient.py           # The main client-side classes for the Apollo bridge.
            ApolloBridgeClient_Parser.py    # Methods to encode and decode ProtoBuf messages as required by Apollo.

To get started, take a look at the general architecture of the CoSIM framework in the next section.