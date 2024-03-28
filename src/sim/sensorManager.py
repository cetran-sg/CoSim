
import carla

class sensorManager():
    def __init__(self, CARLA_HOST, CARLA_PORT):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == 'hero':
                    self.ego_vehicle = actor
        
        self.camera_count = 1
        self.lidar_count = 1

        self.camera_location = carla.Location(x=0.7, z=1.7)
        self.camera_transform = carla.Transform(self.camera_location)

        self.camera2_location = carla.Location(x=0.48, z=1.22)
        self.camera2_transform = carla.Transform(self.camera2_location)
        self.lidar_location = carla.Location(0,0,1.8)
        self.lidar_rotation = carla.Rotation(0,0,0)
        self.lidar_transform = carla.Transform(self.lidar_location,self.lidar_rotation)

    def spawn_cameras(self):
        
        CAM_IMG_SIZE_X = 1280
        CAM_IMG_SIZE_Y = 720
        CAM_FOV = 70
        CAM_SENSOR_TICK = 0.15
        
        # Find the blueprint of the sensor.
        cam1bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        cam1bp.set_attribute('image_size_x', str(CAM_IMG_SIZE_X))
        cam1bp.set_attribute('image_size_y', str(CAM_IMG_SIZE_Y))
        cam1bp.set_attribute('fov', str(CAM_FOV))
        cam1bp.set_attribute('sensor_tick', str(CAM_SENSOR_TICK))
        camera = self.world.spawn_actor(cam1bp, self.camera_transform, attach_to=self.ego_vehicle)
        return camera

    def spawn_lidars(self):     

        LIDAR_CHANNELS = 16
        LIDAR_POINTS_P_SECOND = 1500000
        LIDAR_ROTATION_FREQ = 300
        LIDAR_LOWER_FOV= -15
        LIDAR_UPPER_FOV = 3
        LIDAR_RANGE = 50
        LIDAR_SENSOR_TICK = 0.1
        
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(LIDAR_CHANNELS))
        lidar_bp.set_attribute('points_per_second',str(LIDAR_POINTS_P_SECOND))
        lidar_bp.set_attribute('rotation_frequency',str(LIDAR_ROTATION_FREQ))
        lidar_bp.set_attribute('lower_fov',str(LIDAR_LOWER_FOV))
        lidar_bp.set_attribute('upper_fov',str(LIDAR_UPPER_FOV))
        lidar_bp.set_attribute('range',str(LIDAR_RANGE))
        lidar_bp.set_attribute('sensor_tick', str(LIDAR_SENSOR_TICK))

        lidar = self.world.spawn_actor(lidar_bp,self.lidar_transform,attach_to=self.ego_vehicle)

        return lidar
