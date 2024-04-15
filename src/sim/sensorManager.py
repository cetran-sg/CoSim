import carla
import sensorConfig as sCONFIG

class sensorManager():
    def __init__(self, CARLA_HOST, CARLA_PORT):
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == 'hero':
                    self.ego_vehicle = actor
        
        self.camera_count = sCONFIG.CAMERA_COUNT
        self.lidar_count = sCONFIG.LIDAR_COUNT

        self.camera_location = carla.Location(sCONFIG.CAMERA_LOCATION_X, sCONFIG.CAMERA_LOCATION_Y, sCONFIG.CAMERA_LOCATION_Z)
        self.camera_transform = carla.Transform(self.camera_location)

        self.lidar_location = carla.Location(sCONFIG.LIDAR_LOCATION_X, sCONFIG.LIDAR_LOCATION_Y, sCONFIG.LIDAR_LOCATION_Z)
        # Rotation angles in Carla format (pitch, yaw, roll)
        self.lidar_rotation = carla.Rotation(sCONFIG.LIDAR_ROTATION_P, sCONFIG.LIDAR_ROTATION_Y, sCONFIG.LIDAR_ROTATION_R)
        self.lidar_transform = carla.Transform(self.lidar_location,self.lidar_rotation)

    def spawn_cameras(self):
        
        # Find the blueprint of the sensor.
        cam1bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        cam1bp.set_attribute('image_size_x', str(sCONFIG.CAM_IMG_SIZE_X))
        cam1bp.set_attribute('image_size_y', str(sCONFIG.CAM_IMG_SIZE_Y))
        cam1bp.set_attribute('fov', str(sCONFIG.CAM_FOV))
        cam1bp.set_attribute('sensor_tick', str(sCONFIG.CAM_SENSOR_TICK))
        camera = self.world.spawn_actor(cam1bp, self.camera_transform, attach_to=self.ego_vehicle)
        return camera

    def spawn_lidars(self):     
        
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(sCONFIG.LIDAR_CHANNELS))
        lidar_bp.set_attribute('points_per_second',str(sCONFIG.LIDAR_POINTS_P_SECOND))
        lidar_bp.set_attribute('rotation_frequency',str(sCONFIG.LIDAR_ROTATION_FREQ))
        lidar_bp.set_attribute('lower_fov',str(sCONFIG.LIDAR_LOWER_FOV))
        lidar_bp.set_attribute('upper_fov',str(sCONFIG.LIDAR_UPPER_FOV))
        lidar_bp.set_attribute('range',str(sCONFIG.LIDAR_RANGE))
        lidar_bp.set_attribute('sensor_tick', str(sCONFIG.LIDAR_SENSOR_TICK))

        lidar = self.world.spawn_actor(lidar_bp,self.lidar_transform,attach_to=self.ego_vehicle)

        return lidar
