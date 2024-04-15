# IP and port of the Carla Python API server
CARLA_HOST = 'localhost'
CARLA_PORT = 2000

# IP and port of the Apollo machine
APOLLO_HOST = '127.0.0.1'
APOLLO_PORT = 9999

# Enable ego vehicle (disable if you want to create and test scenarios without ego)
egoOn=True

# Name of the Carla map
worldID = '0712'

# Ego vehicle identifier from Carla blueprint library
carla_ego_name = 'vehicle.lincoln.mkz_2017'

# Ego spawn position and yaw
ego_spawn_pos_x = -7.022+118.04-7.5
ego_spawn_pos_y = 17.865-92.56+10
ego_spawn_pos_z = 3
ego_spawn_rot_yaw = 322.5

# # Ego spawn position and yaw
# ego_spawn_pos_x = -529.872
# ego_spawn_pos_y = -206.508
# ego_spawn_pos_z = 3
# ego_spawn_rot_yaw = 67.5

## Flags for enabling or disabling various features
# Flag to play scenario defined in scenarioManager
scenarioPlayerFlag = True

# Flag to send actor ground truth data to Apollo
actorReaderFlag = True

# Flag to send ego vehicle data to Apollo
egoReaderFlag = True

# Flag to write ego control command data from Apollo
egoWriterFlag = True

# Flags to enable/disable camera and LiDAR sensor output
# If camera is enabled, in the Apollo docker launch: cyber_launch start modules/drivers/tools/image_decompress/launch/image_decompress.launch
camReaderFlag = False
lidarReaderFlag = False

'''
The Carla and Apollo maps are generated from Mathworks RoadRunner.
Carla coordinate system is the same as the exported map coordinates except with -y
Apollo map has an arbitrary offset which is specified below and is applied to the localization messages.

To find this offset, create a 5m long road from (0,0) to (0,5) in RoadRunner. Export the OpenDrive (.xodr) and Apollo map (.txt, .bin) formats.
From the .xodr, find the ID of the road. Find the same road ID in the Apollo .txt file and note the x and y values. These values are the offset. 
'''
APOLLO_MAP_OFFSET_X = 833964.71433961485
APOLLO_MAP_OFFSET_Y = 9999787.0663516354