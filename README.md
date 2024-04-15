# CoSim

### CETRAN Open-Source Simulation framework

This repository contains code for the CoSim framework. The CoSim framework is a reference simulation framework for the virtual testing of autonomous (automated) vehicles built on top of the [Carla simulator](https://github.com/carla-simulator/carla). It also includes a reference bridge to [Baidu Apollo 8.0](https://github.com/ApolloAuto/apollo/tree/r8.0.0).

## Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.
    site/         # Documentation website HTML and other files
    src/
        ads/
            apolloBridgeServer.py           # The main server-side script for the Apollo bridge. Run inside Apollo Docker.
            cyberReader.py                  # Module to read control commands from Apollo CyberRT.
            cyberWriter.py                  # Module to write simulation messages into Apollo CyberRT.
        sim/
            modules/                        # Directory containing protobuf modules for Apollo messages
            agents/                         # Directory containing some Carla Python API modules for scenario generation and execution
            cosimManager.py                 # The main script for the CoSIM simulation framework.
            bridgeClient.py                 # The main client-side classes for the Apollo bridge.
            apolloEncode.py                 # Methods to encode and decode ProtoBuf messages as required by Apollo.
            sensorManager.py                # Module to define ego vehicle sensors and placement
            scenarioManager.py              # Module to spawn actors and make them execute scenarios using the Carla Python API
            config.py                       # File containing configuration parameters
            sensorConfig.py                 # File containing configuration parameters pertaining to the sensor setup of the ego vehicle

To get started, take a look at the [code documentation](https://cetran-sg.github.io/CoSim/).

This repository has been developed and is maintained by [CETRAN](https://cetran.sg/).