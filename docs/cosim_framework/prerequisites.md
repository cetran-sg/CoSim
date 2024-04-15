# Prerequisites

## Software requirements

Before trying out the CoSim framework yourself, ensure that the following software packages are installed.

### Simulation machine

 - Operating system: Windows 10/11 or Ubuntu 20.04
 - Python 3.8.10
   - [NumPy](https://pypi.org/project/numpy/)
   - [opencv-python](https://pypi.org/project/opencv-python/)
   - [pandas](https://pypi.org/project/pandas/)
   - [transforms3d](https://pypi.org/project/transforms3d/)
   - [Netstruct](https://pypi.org/project/netstruct/)
   - [protobuf 3.14](https://pypi.org/project/protobuf/3.14.0/)

### ADS machine
 - Operating system: Ubuntu 20.04
 - Apollo 8.0
    - Install prerequisites for Apollo as described [here](https://github.com/ApolloAuto/apollo/blob/master/docs/01_Installation%20Instructions/prerequisite_software_installation_guide.md)
    - Install Apollo as described [here](https://github.com/ApolloAuto/apollo/blob/master/docs/01_Installation%20Instructions/apollo_software_installation_guide.md)
 - Python 3.8.10
   - [netstruct](https://pypi.org/project/netstruct/)

#### Apollo flag configuration to enable planning module with Carla bridge

On the Apollo machine, set the `enable_map_reference_unify` flag as 'false' in `apollo/modules/common/configs/config_gflags.cc`

Once set as 'false', it should look like this:

`DEFINE_bool(enable_map_reference_unify, false,
            "enable IMU data convert to map reference");`

Now, rebuild Apollo using `./apollo.sh build_opt_gpu`.

## Hardware recommendations

### Simulation machine

 - CPU - A modern AMD or Intel CPU with good single and multithreaded performance
 - GPU - A modern AMD or Nvidia GPU supporting Vulkan and DX12
 - Memory - At least 16GB
 - Storage - At least 500GB
 - Network - At least 1GbE

### ADS machine

 - CPU - At least an 8-core AMD or Intel CPU with good single and multithreaded performance
 - GPU - NVIDIA Turing GPU or AMD GFX9/RDNA/CDNA GPU
 - Memory - At least 16GB
 - Storage - At least 500GB
 - Network - At least 1GbE
 
