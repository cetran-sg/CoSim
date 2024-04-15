# Bridge Client

The bridge client gets information about the simulation world, actors and the ego vehicle from the Carla simulation server. It also receives the control commands as supplied by the ADS (Automated Driving System) and applies it to the simulated ego vehicle.

The bridge client consists of two files, bridgeClient.py and apolloEncode.py. The former handles communication with the bridge server and the latter handles encoding of messages into formats compatible with the Baidu Apollo ADS.

## bridgeClient.py
::: src.sim.bridgeClient

## apolloEncode.py
::: src.sim.apolloEncode