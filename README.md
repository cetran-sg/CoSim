# CoSim

## CETRAN open-source Simulation framework

This repository contains code for the CoSim framework. The CoSim framework is a reference simulation framework for the virtual testing of autonomous (automated) vehicles built on top of the [Carla simulator](https://github.com/carla-simulator/carla). It also includes a reference bridge to [Baidu Apollo 8.0](https://github.com/ApolloAuto/apollo/tree/r8.0.0).

### Project layout

    mkdocs.yml                              # The configuration file.
    docs/
        index.md                            # The documentation homepage.
        ...                                 # Other markdown pages, images and other files.
    src/
        ads/
            apolloBridgeServer.py           # The main server-side script for the Apollo bridge. Run inside Apollo Docker.
            cyberReader.py                  # Module to read control commands from Apollo CyberRT.
            cyberWriter.py                  # Module to write simulation messages into Apollo CyberRT.
        sim/
            modules/                        # Directory containing protobuf modules for Apollo messages
            agents/                         # Directory containing some Carla Python API modules for scenario generation and execution
            cosimManager.py                 # The main script for the CoSim framework.
            bridgeClient.py                 # The main client-side classes for the Apollo bridge.
            apolloEncode.py                 # Methods to encode and decode ProtoBuf messages as required by Apollo.
            sensorManager.py                # Module to define ego vehicle sensors and placement
            scenarioManager.py              # Module to spawn actors and make them execute scenarios using the Carla Python API
            config.py                       # File containing general configuration parameters.
            sensorConfig.py                 # File containing configuration parameters pertaining to the sensor setup of the ego vehicle.

## Documentation

To get started, take a look at the [code documentation](https://cetran-sg.github.io/CoSim/). The documentation contains an overview of the framework along with references to the various functions and modules contained within.

## License

CoSim has been provided under [the MIT License](https://github.com/cetran-sg/CoSim/blob/main/LICENSE.txt).

## Disclaimer

This repository and the code within is intended only as a reference for virtual simulation-based autonomous vehicle (AV) testing and related implementations/processes.

## Connect with us

- [Questions, suggestions, improvements or other issues?](https://github.com/cetran-sg/CoSim/issues)
- [CETRAN](https://cetran.sg/) - Centre of Excellence for Testing & Research of Autonomous Vehicles - NTU
- [NTU](https://www.ntu.edu.sg/) - Nanyang Technological University, Singapore

The CoSim framework has been developed and is maintained by [CETRAN](https://cetran.sg/).

## Citation

If you use CoSim, please cite this repository:

```
@software{Vijay_CoSim_2024,
author = {Vijay, Roshan and Piazzoni, Andrea and De Boer, Niels},
doi = {10.5281/zenodo.10976511},
month = apr,
title = {{CoSim}},
url = {https://github.com/cetran-sg/CoSim},
version = {1.0},
year = {2024}
}
```