from tkinter import *
from socket import *
import threading
import airsim
from airsim.client import *
import time
import numpy as np


def plot_target(pose, client, color=None):
    if color is None:
        color = [1.0, 0.0, 0.0, 1.0]
    pos = [pose.position.x_val, pose.position.y_val, pose.position.z_val]
    a = [airsim.Vector3r(pos[0] - 20, pos[1], -50)]
    b = [airsim.Vector3r(pos[0], pos[1] - 20, -50)]
    c = [airsim.Vector3r(pos[0] + 20, pos[1], -50)]
    d = [airsim.Vector3r(pos[0], pos[1] + 20, -50)]
    client.simPlotLineList(a + b + b + c + c + d + d + a, thickness=250.0, duration=0.2,
                              color_rgba=color,
                              is_persistent=False)


class VehicleDict:
    def __init__(self,  addr, socket_client):
        self.addr = addr
        self.socket_client = socket_client
        self.airsim_name = None
        self.client = None
        self.pose_client = None
        self.blueprint_client = None
        self.send_flag = 0


def send_msg(vehicle_class, plot_flag=False):
    while True:
        if vehicle_class.send_flag == 0:
            pos = vehicle_class.pose_client.simGetObjectPose(vehicle_class.airsim_name)
            d_pos = pos.position
            euler = np.rad2deg(airsim.to_eularian_angles(pos.orientation))
            if plot_flag:
                plot_target(pos, vehicle_class.blueprint_client)
            msg = '_' + str(d_pos.x_val * 100) + '_' + str(d_pos.y_val * 100) + '_' + str(-d_pos.z_val * 100) + '_' + \
                  str(euler[0]) + '_' + str(euler[1]) + '_' + str(euler[2])
            # print(msg)
            vehicle_class.socket_client.sendall(msg.encode('utf-8'))

        else:
            vehicle_class.send_flag = 0

        time.sleep(0.1)

class CustomAirsimClient:
    def __init__(self, ip_list, socket_server, plot_flag=False):
        self.airsim_client_list = []
        self.plot_flag = plot_flag
        self.client = airsim.MultirotorClient('127.0.0.1')
        self.client.ip = '127.0.0.1'
        self.blueprint_ip = '127.0.0.1'
        self.t_main = []
        self.airsim_vehicle_dict = {}
        # socket_server -> socket()
        self.socket_server = socket_server
        self.buff_size = 2048
        for each in ip_list:
            self.airsim_client_list.append(airsim.MultirotorClient(each))
            self.airsim_client_list[-1].ip = each
            self.airsim_vehicle_dict[self.airsim_client_list[-1]] = self.airsim_client_list[-1].listVehicles()
        self.airsim_client_list.append(self.client)
        self.airsim_vehicle_dict[self.airsim_client_list[-1]] = self.airsim_client_list[-1].listVehicles()
        # 连接的ip和端口号
        self.conn_list = []
        # 字典 key：ip+port value: socket_client
        self.conn_dt = {}
        self.drone_num = 0
        # 已分配好airsim节点的blueprint vehicle
        self.assigned_blueprint = []
        # 字典 key: blueprint name  value: vehicle_dict class
        self.vehicle_dict = {}
        self.t = threading.Thread(target=self.recs, args=())
        self.t.start()
        self.t2 = threading.Thread(target=self.vehicle_assign, args=())
        self.t2.start()

    def vehicle_assign(self):
        n = 1
        while True:
            flag = 0
            if len(self.assigned_blueprint) < self.drone_num:
                for each in self.vehicle_dict:
                    if each not in self.assigned_blueprint:
                        for tmp_client in self.airsim_vehicle_dict:
                            for str_tmp in self.airsim_vehicle_dict[tmp_client]:
                                if str_tmp[0:2] == each[0:2]:
                                    # tmp_client.enableApiControl(True, str_tmp)
                                    self.vehicle_dict[each].airsim_name = str_tmp
                                    self.vehicle_dict[each].client = tmp_client
                                    self.vehicle_dict[each].pose_client = airsim.MultirotorClient(
                                        self.vehicle_dict[each].client.ip)
                                    self.vehicle_dict[each].blueprint_client = airsim.MultirotorClient(self.blueprint_ip)
                                    self.vehicle_dict[each].pose_client.enableApiControl(
                                        True,  self.vehicle_dict[each].airsim_name)
                                    self.assigned_blueprint.append(each)
                                    self.airsim_vehicle_dict[tmp_client].remove(str_tmp)
                                    n = 1
                                    self.t_main.append(threading.Thread(target=send_msg,
                                                                        args=[self.vehicle_dict[each], self.plot_flag]))
                                    self.t_main[-1].start()
                                    flag = 1
                                    break
                            if flag:
                                break
                    if flag:
                        break
            else:
                n *= 2
                if n == 4:
                    break
                time.sleep(n)
                # threading.Timer()

    def get_info(self, vehicle_name, msg='neighbor'):
        """ msg = neighbor、 reset 、 crash, etc."""
        self.vehicle_dict[vehicle_name].send_flag = 1
        self.vehicle_dict[vehicle_name].socket_client.sendall(msg.encode('utf-8'))
        recv_data = self.vehicle_dict[vehicle_name].socket_client.recv(self.buff_size).decode('utf-8')
        return recv_data

    def recs(self):
        while True:
            clientsock, client_address = self.socket_server.accept()
            if client_address not in self.conn_dt:
                self.conn_list.append(client_address)
                self.conn_dt[client_address] = clientsock
                recv_data = clientsock.recv(self.buff_size).decode('utf-8')
                self.vehicle_dict[recv_data] = VehicleDict(client_address, clientsock)

                # gui.listBox.insert(END, clientaddress)
                self.drone_num += 1
                print(recv_data, 'connect from:', client_address)

    # ----------------------------------- Common vehicle APIs ---------------------------------------------
    def reset(self):
        """
        Reset the vehicle to its original starting state
        Note that you must call `enableApiControl` and `armDisarm` again after the call to reset
        """
        for each in self.airsim_client_list:
            each.reset()

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout
        """
        a = []
        for each in self.airsim_client_list:
            a.append(each.ping())
        return a

    def getClientVersion(self, call_flag =False):
        return 1  # sync with C++ client

    def getServerVersion(self):
        a = []
        for each in self.airsim_client_list:
            a.append(each.getServerVersion())
        return a

    def getMinRequiredServerVersion(self):
        return 1  # sync with C++ client

    def getMinRequiredClientVersion(self):
        a = []
        for each in self.airsim_client_list:
            a.append(each.getMinRequiredClientVersion())
        return a

        # basic flight control

    def enableApiControl(self, is_enabled, vehicle_name=''):
        """
        Enables or disables API control for vehicle corresponding to vehicle_name
        Args:
            is_enabled (bool): True to enable, False to disable API control
            vehicle_name (str, optional): Name of the vehicle to send this command to
        """
        self.vehicle_dict[vehicle_name].client.enableApiControl(
            is_enabled, self.vehicle_dict[vehicle_name].airsim_name)

    def isApiControlEnabled(self, vehicle_name=' '):
        """
        Returns true if API control is established.
        If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, `isApiControlEnabled` should return true.
        Args:
            vehicle_name (str, optional): Name of the vehicle
        Returns:
            bool: If API control is enabled
        """
        return self.vehicle_dict[vehicle_name].client.isApiControlEnabled(self.vehicle_dict[vehicle_name].airsim_name)

    def armDisarm(self, arm, vehicle_name=''):
        """
        Arms or disarms vehicle
        Args:
            arm (bool): True to arm, False to disarm the vehicle
            vehicle_name (str, optional): Name of the vehicle to send this command to
        Returns:
            bool: Success
        """

        return self.vehicle_dict[vehicle_name].client.armDisarm(arm, self.vehicle_dict[vehicle_name].airsim_name)

    def simPause(self, is_paused):
        """
        Pauses simulation
        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        a = []
        for each in self.airsim_client_list:
            a.append(each.simPause(is_paused))
        # a.append(self.client.simPause(is_paused))
        return a

    def simIsPause(self):
        """
        Returns true if the simulation is paused
        Returns:
            bool: If the simulation is paused
        """
        a = []
        for each in self.airsim_client_list:
            a.append(each.simIsPause())
        return a

    def simContinueForTime(self, seconds):
        """
        Continue the simulation for the specified number of seconds
        Args:
            seconds (float): Time to run the simulation for
        """
        for each in self.airsim_client_list:
            each.simContinueForTime(seconds)

    def simContinueForFrames(self, frames):
        """
        Continue (or resume if paused) the simulation for the specified number of frames, after which the simulation will be paused.
        Args:
            frames (int): Frames to run the simulation for
        """
        for each in self.airsim_client_list:
            each.simContinueForFrames(frames)

    def getHomeGeoPoint(self, vehicle_name=''):
        """
        Get the Home location of the vehicle
        Args:
            vehicle_name (str, optional): Name of vehicle to get home location of
        Returns:
            GeoPoint: Home location of the vehicle
        """
        return self.vehicle_dict[vehicle_name].client.getHomeGeoPoint(self.vehicle_dict[vehicle_name].airsim_name)

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        """
        if all(self.ping()):
            print("Connected!")
        else:
            print("Ping returned false!")

    def simSetLightIntensity(self, light_name, intensity):
        """
        Change intensity of named light
        Args:
            light_name (str): Name of light to change
            intensity (float): New intensity value
        Returns:
            bool: True if successful, otherwise False
        """
        return self.client.simSetLightIntensity(light_name, intensity)

    def simSwapTextures(self, tags, tex_id=0, component_id=0, material_id=0):
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            tags (str): string of "," or ", " delimited tags to identify on which actors to perform the swap
            tex_id (int, optional): indexes the array of textures assigned to each actor undergoing a swap
                                    If out-of-bounds for some object's texture set, it will be taken modulo the number of textures that were available
            component_id (int, optional):
            material_id (int, optional):
        Returns:
            list[str]: List of objects which matched the provided tags and had the texture swap perfomed
        """
        return self.client.simSwapTextures(tags, tex_id, component_id, material_id)

    def simSetObjectMaterial(self, object_name, material_name):
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): name of object to set material for
            material_name (str): name of material to set for object
        Returns:
            bool: True if material was set
        """
        return self.client.simSetObjectMaterial(object_name, material_name)

    def simSetObjectMaterialFromTexture(self, object_name, texture_path):
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): name of object to set material for
            texture_path (str): path to texture to set for object
        Returns:
            bool: True if material was set
        """
        return self.client.simSetObjectMaterialFromTexture(object_name, texture_path)

        # time-of-day control
        # time - of - day control

    def simSetTimeOfDay(self, is_enabled, start_datetime="", is_start_datetime_dst=False, celestial_clock_speed=1,
                        update_interval_secs=60, move_sun=True):
        """
        Control the position of Sun in the environment
        Sun's position is computed using the coordinates specified in `OriginGeopoint` in settings for the date-time specified in the argument,
        else if the string is empty, current date & time is used
        Args:
            is_enabled (bool): True to enable time-of-day effect, False to reset the position to original
            start_datetime (str, optional): Date & Time in %Y-%m-%d %H:%M:%S format, e.g. `2018-02-12 15:20:00`
            is_start_datetime_dst (bool, optional): True to adjust for Daylight Savings Time
            celestial_clock_speed (float, optional): Run celestial clock faster or slower than simulation clock
                                                     E.g. Value 100 means for every 1 second of simulation clock, Sun's position is advanced by 100 seconds
                                                     so Sun will move in sky much faster
            update_interval_secs (float, optional): Interval to update the Sun's position
            move_sun (bool, optional): Whether or not to move the Sun
        """
        self.client.simSetTimeOfDay(
            is_enabled, start_datetime, is_start_datetime_dst, celestial_clock_speed, update_interval_secs, move_sun)

        # weather

    def simEnableWeather(self, enable):
        """
        Enable Weather effects. Needs to be called before using `simSetWeatherParameter` API
        Args:
            enable (bool): True to enable, False to disable
        """
        self.client.simEnableWeather(enable)

    def simSetWeatherParameter(self, param, val):
        """
        Enable various weather effects
        Args:
            param (WeatherParameter): Weather effect to be enabled
            val (float): Intensity of the effect, Range 0-1
        """
        self.client.simSetWeatherParameter(param, val)

        # camera control
        # simGetImage returns compressed png in array of bytes
        # image_type uses one of the ImageType members

    def simGetImage(self, camera_name, image_type, vehicle_name='', external=False):
        """
        Get a single image
        Returns bytes of png format image which can be dumped into abinary file to create .png image
        `string_to_uint8_array()` can be used to convert into Numpy unit8 array
        See https://microsoft.github.io/AirSim/image_apis/ for details
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Name of the vehicle with the camera
            external (bool, optional): Whether the camera is an External Camera
        Returns:
            Binary string literal of compressed png image
        """
        return self.vehicle_dict[vehicle_name]\
            .client.simGetImage(
            camera_name, image_type, self.vehicle_dict[vehicle_name].airsim_name, external)
        # camera control
        # simGetImage returns compressed png in array of byteslLL
        # image_type uses one of the ImageType members

    def simGetImages(self, requests, vehicle_name='', external=False):
        """
        Get multiple images
        See https://microsoft.github.io/AirSim/image_apis/ for details and examples
        Args:
            requests (list[ImageRequest]): Images required
            vehicle_name (str, optional): Name of vehicle associated with the camera
            external (bool, optional): Whether the camera is an External Camera
        Returns:
            list[ImageResponse]:
        """
        return self.vehicle_dict[vehicle_name] \
            .client.simGetImages(requests, self.vehicle_dict[vehicle_name].airsim_name, external)
        # CinemAirSim

    def simGetPresetLensSettings(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name] \
            .client.simGetPresetLensSettings(camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetLensSettings(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name] \
            .client.simGetLensSettings(camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetPresetLensSettings(self, preset_lens_settings, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name] \
            .client.simSetPresetLensSettings(preset_lens_settings, camera_name,
                                             self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetPresetFilmbackSettings(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name] \
            .client.simGetPresetFilmbackSetting(camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetPresetFilmbackSettings(self, preset_film_back_settings, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name] \
            .client.simSetPresetFilmbackSettings(preset_film_back_settings, camera_name,
                                                 self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetFilmbackSettings(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name]\
            .client.simGetFilmbackSettings(camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetFilmbackSettings(self, sensor_width, sensor_height, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name].client.simSetFilmbackSettings(
            sensor_width, sensor_height, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetFocalLength(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name].client.simGetFocalLength(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetFocalLength(self, focal_length, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name].client.simSetFocalLength(
            focal_length, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simEnableManualFocus(self, enable, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name].client.simEnableManualFocus(
            enable, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetFocusDistance(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name].client.simGetFocusDistance(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetFocusDistance(self, focus_distance, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name].client.simSetFocusDistance(
            focus_distance, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetFocusAperture(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name].client.simGetFocusAperture(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetFocusAperture(self, focus_aperture, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name].client.simSetFocusAperture(
            focus_aperture, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simEnableFocusPlane(self, enable, camera_name, vehicle_name='', external=False):
        self.vehicle_dict[vehicle_name].client.simEnableFocusPlane(
            enable, camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetCurrentFieldOfView(self, camera_name, vehicle_name='', external=False):
        return self.vehicle_dict[vehicle_name].client.simGetCurrentFieldOfView(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

        # End CinemAirSim
    def simTestLineOfSightToPoint(self, point, vehicle_name=''):
        """
        Returns whether the target point is visible from the perspective of the inputted vehicle
        Args:
            point (GeoPoint): target point
            vehicle_name (str, optional): Name of vehicle
        Returns:
            [bool]: Success
        """
        return self.vehicle_dict[vehicle_name].client.simTestLineOfSightToPoint(
            point, self.vehicle_dict[vehicle_name].airsim_name)

    def simTestLineOfSightBetweenPoints(self, point1, point2):
        """
        Returns whether the target point is visible from the perspective of the source point
        Args:
            point1 (GeoPoint): source point
            point2 (GeoPoint): target point
        Returns:
            [bool]: Success
        """
        return self.client.simTestLineOfSightBetweenPoints(point1, point2)

    def simGetWorldExtents(self):
        """
        Returns a list of GeoPoints representing the minimum and maximum extents of the world
        Returns:
            list[GeoPoint]
        """
        return self.client.simGetWorldExtents()

    def simRunConsoleCommand(self, command):
        """
        Allows the client to execute a command in Unreal's native console, via an API.
        Affords access to the countless built-in commands such as "stat unit", "stat fps", "open [map]", adjust any config settings, etc. etc.
        Allows the user to create bespoke APIsSS very easily, by adding a custom event to the level blueprint, and then calling the console command "ce MyEventName [args]". No recompilation of AirSim needed!
        Args:
            command ([string]): Desired Unreal Engine Console command to run
        Returns:
            [bool]: Success
        """
        return self.client.simRunConsoleCommand(command)

        # gets the static meshes in the unreal scene

    def simGetMeshPositionVertexBuffers(self):
        """
        Returns the static meshes that make up the scene
        See https://microsoft.github.io/AirSim/meshes/ for details and how to use this
        Returns:
            list[MeshPositionVertexBuffersResponse]:
        """
        return self.client.simGetMeshPositionVertexBuffers()

    def simGetCollisionInfo(self, vehicle_name=''):
        """
        Args:
            vehicle_name (str, optional): Name of the Vehicle to get the info of
        Returns:
            CollisionInfo:
        """
        return self.vehicle_dict[vehicle_name].client.simGetCollisionInfo(self.vehicle_dict[vehicle_name].airsim_name)

    def simSetVehiclePose(self, pose, ignore_collision, vehicle_name=''):
        """
        Set the pose of the vehicle
        If you don't want to change position (or orientation) then just set components of position (or orientation) to floating point nan values
        Args:
            pose (Pose): Desired Pose pf the vehicle
            ignore_collision (bool): Whether to ignore any collision or not
            vehicle_name (str, optional): Name of the vehicle to move
        """
        self.vehicle_dict[vehicle_name].client.simSetVehiclePose(pose, ignore_collision,
                                                                 self.vehicle_dict[vehicle_name].airsim_name)

    def simGetVehiclePose(self, vehicle_name=''):
        """
        The position inside the returned Pose is in the frame of the vehicle's starting point

        Args:
            vehicle_name (str, optional): Name of the vehicle to get the Pose of
        Returns:
            Pose:
        """
        return self.vehicle_dict[vehicle_name].client.simGetVehiclePose(self.vehicle_dict[vehicle_name].airsim_name)

    def simSetTraceLine(self, color_rgba, thickness=1.0, vehicle_name=''):
        """
        Modify the color and thickness of the line when Tracing is enabled
        Tracing can be enabled by pressing T in the Editor or setting `EnableTrace` to `True` in the Vehicle Settings
        Args:
            color_rgba (list): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of the line
            vehicle_name (string, optional): Name of the vehicle to set Trace line values for
        """
        self.vehicle_dict[vehicle_name].client.simSetTraceLine(color_rgba,
                                                               thickness, self.vehicle_dict[vehicle_name].airsim_name)

    def simGetObjectPose(self, object_name):
        """
        The position inside the returned Pose is in the world frame
        Args:
            object_name (str): Object to get the Pose of
        Returns:
            Pose:
        """
        if object_name[0:2] == 'cf' or object_name[0:2] == 'uv' or object_name[0:2] == 'fw':
            return self.vehicle_dict[object_name].client.simGetObjectPose(self.vehicle_dict[object_name].airsim_name)
        else:
            return self.client.simGetObjectPose(object_name)

    def simSetObjectPose(self, object_name, pose, teleport=True):
        """
        Set the pose of the object(actor) in the environment
        The specified actor must have Mobility set to movable, otherwise there will be undefined behaviour.
        See https://www.unrealengine.com/en-US/blog/moving-physical-objects for details on how to set Mobility and the effect of Teleport parameter
        Args:
            object_name (str): Name of the object(actor) to move
            pose (Pose): Desired Pose of the object
            teleport (bool, optional): Whether to move the object immediately without affecting their velocity
        Returns:
            list[bool]: If the move was successful
        """
        a = [self.client.simSetObjectPose(object_name, pose, teleport)]
        for each in self.airsim_client_list:
            if object_name[0:2] == 'cf' or object_name[0:2] == 'uv' or object_name[0:2] == 'fw':
                a.append(each.simSetObjectPose(self.vehicle_dict[object_name].airsim_name, pose, teleport))
            else:
                a.append(each.simSetObjectPose(object_name, pose, teleport))
        return a

    def simGetObjectScale(self, object_name):
        """
        Gets scale of an object in the world
        Args:
            object_name (str): Object to get the scale of
        Returns:
            airsim.Vector3r: Scale
        """
        return self.client.simGetObjectScale(object_name)

    def simSetObjectScale(self, object_name, scale_vector):
        """
        Sets scale of an object in the world
        Args:
            object_name (str): Object to set the scale of
            scale_vector (airsim.Vector3r): Desired scale of object
        Returns:
            List[bool]: True if scale change was successful
        """
        a = [self.client.simSetObjectScale(object_name, scale_vector)]
        for each in self.airsim_client_list:
            if object_name[0:2] == 'cf' or object_name[0:2] == 'uv' or object_name[0:2] == 'fw':
                a.append(each.simSetObjectScale(self.vehicle_dict[object_name].airsim_name, scale_vector))
            else:
                a.append(each.simSetObjectScale(object_name, scale_vector))
        return a

    def simListSceneObjects(self, name_regex='.*'):
        """
        Lists the objects present in the environment
        Default behaviour is to list all objects, regex can be used to return smaller list of matching objects or actors
        Args:
            name_regex (str, optional): String to match actor names against, e.g. "Cylinder.*"
        Returns:
            list[str]: List containing all the names
        """
        return self.client.simListSceneObjects(name_regex)

    def simLoadLevel(self, level_name):
        """
        Loads a level specified by its name
        Args:
            level_name (str): Name of the level to load
        Returns:
            bool: True if the level was successfully loaded
        """
        a = [self.client.simLoadLevel(level_name)]
        for each in self.airsim_client_list:
            a.append(each.simLoadLevel(level_name))
        return a

    def simListAssets(self):
        """
        Lists all the assets present in the Asset Registry
        Returns:
            list[str]: Names of all the assets
        """
        return self.client.simListAssets()

    def simSpawnObject(self, object_name, asset_name, pose, scale, physics_enabled=False, is_blueprint=False):
        """Spawned selected object in the world
        Args:
            object_name (str): Desired name of new object
            asset_name (str): Name of asset(mesh) in the project database
            pose (airsim.Pose): Desired pose of object
            scale (airsim.Vector3r): Desired scale of object
            physics_enabled (bool, optional): Whether to enable physics for the object
            is_blueprint (bool, optional): Whether to spawn a blueprint or an actor
        Returns:
            str: Name of spawned object, in case it had to be modified
        """

        a = [self.client.simSpawnObject(object_name, asset_name, pose, scale, physics_enabled, is_blueprint)]
        for each in self.airsim_client_list:
            a.append(each.simSpawnObject(object_name, asset_name, pose, scale, physics_enabled, is_blueprint))
        return a

    def simDestroyObject(self, object_name):
        """Removes selected object from the world
        Args:
            object_name (str): Name of object to be removed
        Returns:
            bool: True if object is queued up for removal
        """
        a = [self.client.simDestroyObject(object_name)]
        for each in self.airsim_client_list:
            if object_name[0:2] == 'cf' or object_name[0:2] == 'uv' or object_name[0:2] == 'fw':
                a.append(each.simDestroyObject(self.vehicle_dict[object_name].airsim_name))
            else:
                a.append(each.simDestroyObject(object_name))
        return a

    def simSetSegmentationObjectID(self, mesh_name, object_id, is_name_regex=False):
        """
        Set segmentation ID for specific objects
        See https://microsoft.github.io/AirSim/image_apis/#segmentation for details
        Args:
            mesh_name (str): Name of the mesh to set the ID of (supports regex)
            object_id (int): Object ID to be set, range 0-255
                             RBG values for IDs can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt
            is_name_regex (bool, optional): Whether the mesh name is a regex
        Returns:
            bool: If the mesh was found
        """
        return self.client.simSetSegmentationObjectID(mesh_name, object_id, is_name_regex)

    def simGetSegmentationObjectID(self, mesh_name):
        """
        Returns Object ID for the given mesh name
        Mapping of Object IDs to RGB values can be seen at https://microsoft.github.io/AirSim/seg_rgbs.txt
        Args:
            mesh_name (str): Name of the mesh to get the ID of
        """
        return self.client.simGetSegmentationObjectID(mesh_name)

    def simAddDetectionFilterMeshName(self, camera_name, image_type, mesh_name, vehicle_name='', external=False):
        """
        Add mesh name to detect in wild card format
        For example: simAddDetectionFilterMeshName("Car_*") will detect all instance named "Car_*"
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            mesh_name (str): mesh name in wild card format
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simAddDetectionFilterMeshName(
            camera_name, image_type, mesh_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetDetectionFilterRadius(self, camera_name, image_type, radius_cm, vehicle_name='', external=False):
        """
        Set detection radius for all cameras
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            radius_cm (int): Radius in [cm]
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simSetDetectionFilterRadius(
            camera_name, image_type, radius_cm, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simClearDetectionMeshNames(self, camera_name, image_type, vehicle_name='', external=False):
        """
        Clear all mesh names from detection filter
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simClearDetectionMeshNames(
            camera_name, image_type, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetDetections(self, camera_name, image_type, vehicle_name='', external=False):
        """
        Get current detections
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        Returns:
            DetectionInfo array
        """

        return self.vehicle_dict[vehicle_name].client.simGetDetections(
            camera_name, image_type, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simPrintLogMessage(self, message, message_param="", severity=0):
        """
        Prints the specified message in the simulator's window.
        If message_param is supplied, then it's printed next to the message and in that case if this API is called with same message value
        but different message_param again then previous line is overwritten with new line (instead of API creating new line on display).
        For example, `simPrintLogMessage("Iteration: ", to_string(i))` keeps updating same line on display when API is called with different values of i.
        The valid values of severity parameter is 0 to 3 inclusive that corresponds to different colors.
        Args:
            message (str): Message to be printed
            message_param (str, optional): Parameter to be printed next to the message
            severity (int, optional): Range 0-3, inclusive, corresponding to the severity of the message
        """
        self.client.simPrintLogMessage(message, message_param, severity)

    def simGetCameraInfo(self, camera_name, vehicle_name='', external=False):
        """
        Get details about the camera
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        Returns:
            CameraInfo:
        """
        return self.vehicle_dict[vehicle_name].client.simGetCameraInfo(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetDistortionParams(self, camera_name, vehicle_name='', external=False):
        """
        Get camera distortion parameters
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        Returns:
            List (float): List of distortion parameter values corresponding to K1, K2, K3, P1, P2 respectively.
        """
        return self.vehicle_dict[vehicle_name].client.simGetDistortionParams(
            camera_name, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetDistortionParams(self, camera_name, distortion_params, vehicle_name='', external=False):
        """
        Set camera distortion parameters
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            distortion_params (dict): Dictionary of distortion param names and corresponding values
                                        {"K1": 0.0, "K2": 0.0, "K3": 0.0, "P1": 0.0, "P2": 0.0}
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simSetDistortionParams(
            camera_name, distortion_params, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetDistortionParam(self, camera_name, param_name, value, vehicle_name='', external=False):
        """
        Set single camera distortion parameter
        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            param_name (str): Name of distortion parameter
            value (float): Value of distortion parameter
            vehicle_name (str, optional): Vehicle which the camera is associated with
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simSetDistortionParam(
            camera_name, param_name, value, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetCameraPose(self, camera_name, pose, vehicle_name='', external=False):
        """
        - Control the pose of a selected camera
        Args:
            camera_name (str): Name of the camera to be controlled
            pose (Pose): Pose representing the desired position and orientation of the camera
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simSetCameraPose(
            camera_name, pose, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simSetCameraFov(self, camera_name, fov_degrees, vehicle_name='', external=False):
        """
        - Control the field of view of a selected camera
        Args:
            camera_name (str): Name of the camera to be controlled
            fov_degrees (float): Value of field of view in degrees
            vehicle_name (str, optional): Name of vehicle which the camera corresponds to
            external (bool, optional): Whether the camera is an External Camera
        """
        self.vehicle_dict[vehicle_name].client.simSetCameraFov(
            camera_name, fov_degrees, self.vehicle_dict[vehicle_name].airsim_name, external)

    def simGetGroundTruthKinematics(self, vehicle_name=''):
        """
        Get Ground truth kinematics of the vehicle
        The position inside the returned KinematicsState is in the frame of the vehicle's starting point
        Args:
            vehicle_name (str, optional): Name of the vehicle
        Returns:
            KinematicsState: Ground truth of the vehicle
        """
        return self.vehicle_dict[vehicle_name].client.simSetCameraFov(self.vehicle_dict[vehicle_name].airsim_name)

    simGetGroundTruthKinematics.__annotations__ = {'return': KinematicsState}

    def simSetKinematics(self, state, ignore_collision, vehicle_name=''):
        """
        Set the kinematics state of the vehicle
        If you don't want to change position (or orientation) then just set components of position (or orientation) to floating point nan values
        Args:
            state (KinematicsState): Desired Pose pf the vehicle
            ignore_collision (bool): Whether to ignore any collision or not
            vehicle_name (str, optional): Name of the vehicle to move
        """
        self.vehicle_dict[vehicle_name].client.simSetKinematics(
            state, ignore_collision, self.vehicle_dict[vehicle_name].airsim_name)

    def simGetGroundTruthEnvironment(self, vehicle_name=''):
        """
        Get ground truth environment state
        The position inside the returned EnvironmentState is in the frame of the vehicle's starting point
        Args:
            vehicle_name (str, optional): Name of the vehicle
        Returns:
            EnvironmentState: Ground truth environment state
        """
        return self.vehicle_dict[vehicle_name].client.simGetGroundTruthEnvironment(self.vehicle_dict[vehicle_name].airsim_name)

    simGetGroundTruthEnvironment.__annotations__ = {'return': EnvironmentState}

    # sensor APIs
    def getImuData(self, imu_name='', vehicle_name=''):
        """
        Args:
            imu_name (str, optional): Name of IMU to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            ImuData:
        """
        return self.vehicle_dict[vehicle_name].client.getImuData(imu_name, self.vehicle_dict[vehicle_name].airsim_name)

    def getBarometerData(self, barometer_name='', vehicle_name=''):
        """
        Args:
            barometer_name (str, optional): Name of Barometer to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            BarometerData:
        """
        return self.vehicle_dict[vehicle_name].client.getBarometerData(
            barometer_name, self.vehicle_dict[vehicle_name].airsim_name)

    def getMagnetometerData(self, magnetometer_name='', vehicle_name=''):
        """
        Args:
            magnetometer_name (str, optional): Name of Magnetometer to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            MagnetometerData:
        """
        return self.vehicle_dict[vehicle_name].client.getMagnetometerData(
            magnetometer_name, self.vehicle_dict[vehicle_name].airsim_name)

    def getGpsData(self, gps_name='', vehicle_name=''):
        """
        Args:
            gps_name (str, optional): Name of GPS to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            GpsData:
        """
        return self.vehicle_dict[vehicle_name].client.getGpsData(
            gps_name, self.vehicle_dict[vehicle_name].airsim_name)

    def getDistanceSensorData(self, distance_sensor_name='', vehicle_name=''):
        """
        Args:
            distance_sensor_name (str, optional): Name of Distance Sensor to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            DistanceSensorData:
        """
        return self.vehicle_dict[vehicle_name].client.getDistanceSensorData(
            distance_sensor_name, self.vehicle_dict[vehicle_name].airsim_name)

    def getLidarData(self, lidar_name='', vehicle_name=''):
        """
        Args:
            lidar_name (str, optional): Name of Lidar to get data from, specified in settings.json
            vehicle_name (str, optional): Name of vehicle to which the sensor corresponds to
        Returns:
            LidarData:
        """
        return self.vehicle_dict[vehicle_name].client.getLidarData(
            lidar_name, self.vehicle_dict[vehicle_name].airsim_name)

    def simGetLidarSegmentation(self, lidar_name='', vehicle_name=''):
        """
        NOTE: Deprecated API, use `getLidarData()` API instead
        Returns Segmentation ID of each point's collided object in the last Lidar update
        Args:
            lidar_name (str, optional): Name of Lidar sensor
            vehicle_name (str, optional): Name of the vehicle wth the sensor
        Returns:
            list[int]: Segmentation IDs of the objects
        """
        return self.vehicle_dict[vehicle_name].client.simGetLidarSegmentation(
            lidar_name, self.vehicle_dict[vehicle_name].airsim_name)

    # Plotting APIs
    def simFlushPersistentMarkers(self):
        """
        Clear any persistent markers - those plotted with setting `is_persistent=True` in the APIs below
        """
        self.client.simFlushPersistentMarkers()

    def simPlotPoints(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], size=10.0, duration=-1.0, is_persistent=False):
        """
        Plot a list of 3D points in World NED frame
        Args:
            points (list[Vector3r]): List of Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            size (float, optional): Size of plotted point
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.simPlotPoints(points, color_rgba, size, duration, is_persistent)

    def simPlotLineStrip(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=5.0, duration=-1.0,
                         is_persistent=False):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[1] to points[2], ... , points[n-2] to points[n-1]
        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.simPlotLineStrip(points, color_rgba, thickness, duration, is_persistent)

    def simPlotLineList(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=5.0, duration=-1.0,
                        is_persistent=False):
        """
        Plots a line strip in World NED frame, defined from points[0] to points[1], points[2] to points[3], ... , points[n-2] to points[n-1]
        Args:
            points (list[Vector3r]): List of 3D locations of line start and end points, specified as Vector3r objects. Must be even
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.simPlotLineList(points, color_rgba, thickness, duration, is_persistent)

    def simPlotArrows(self, points_start, points_end, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness=5.0, arrow_size=2.0,
                      duration=-1.0, is_persistent=False):
        """
        Plots a list of arrows in World NED frame, defined from points_start[0] to points_end[0], points_start[1] to points_end[1], ... , points_start[n-1] to points_end[n-1]
        Args:
            points_start (list[Vector3r]): List of 3D start positions of arrow start positions, specified as Vector3r objects
            points_end (list[Vector3r]): List of 3D end positions of arrow start positions, specified as Vector3r objects
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            thickness (float, optional): Thickness of line
            arrow_size (float, optional): Size of arrow head
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.simPlotArrows(points_start, points_end,
                                  color_rgba, thickness, arrow_size, duration, is_persistent)

    def simPlotStrings(self, strings, positions, scale=5, color_rgba=[1.0, 0.0, 0.0, 1.0], duration=-1.0):
        """
        Plots a list of strings at desired positions in World NED frame.
        Args:
            strings (list[String], optional): List of strings to plot
            positions (list[Vector3r]): List of positions where the strings should be plotted. Should be in one-to-one correspondence with the strings' list
            scale (float, optional): Font scale of transform name
            color_rgba (list, optional): desired RGBA values from 0.0 to 1.0
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.simPlotStrings(strings, positions, scale, color_rgba, duration)

    def simPlotTransforms(self, poses, scale=5.0, thickness=5.0, duration=-1.0, is_persistent=False):
        """
        Plots a list of transforms in World NED frame.
        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            scale (float, optional): Length of transforms' axes
            thickness (float, optional): Thickness of transforms' axes
            duration (float, optional): Duration (seconds) to plot for
            is_persistent (bool, optional): If set to True, the desired object will be plotted for infinite time.
        """
        self.client.simPlotTransforms(poses, scale, thickness, duration, is_persistent)

    def simPlotTransformsWithNames(self, poses, names, tf_scale=5.0, tf_thickness=5.0, text_scale=10.0,
                                   text_color_rgba=[1.0, 0.0, 0.0, 1.0], duration=-1.0):
        """
        Plots a list of transforms with their names in World NED frame.
        Args:
            poses (list[Pose]): List of Pose objects representing the transforms to plot
            names (list[string]): List of strings with one-to-one correspondence to list of poses
            tf_scale (float, optional): Length of transforms' axes
            tf_thickness (float, optional): Thickness of transforms' axes
            text_scale (float, optional): Font scale of transform name
            text_color_rgba (list, optional): desired RGBA values from 0.0 to 1.0 for the transform name
            duration (float, optional): Duration (seconds) to plot for
        """
        self.client.simPlotTransformsWithNames(poses, names, tf_scale,
                                               tf_thickness, text_scale, text_color_rgba, duration)

    def cancelLastTask(self, vehicle_name=''):
        """
        Cancel previous Async task
        Args:
            vehicle_name (str, optional): Name of the vehicle
        """
        self.vehicle_dict[vehicle_name].client.cancelLastTask(self.vehicle_dict[vehicle_name].airsim_name)

    # Recording APIs
    def startRecording(self):
        """
        Start Recording
        Recording will be done according to the settings
        """
        self.client.startRecording()

    def stopRecording(self):
        """
        Stop Recording
        """
        self.client.stopRecording()

    def isRecording(self):
        """
        Whether Recording is running or not
        Returns:
            bool: True if Recording, else False
        """
        return self.client.isRecording()

    def simSetWind(self, wind):
        """
        Set simulated wind, in World frame, NED direction, m/s
        Args:
            wind (Vector3r): Wind, in World frame, NED direction, in m/s
        """
        a = [self.client.simSetWind(wind)]
        for each in self.airsim_client_list:
            a.append(each.simSetWind(wind))

    def simCreateVoxelGrid(self, position, x, y, z, res, of):
        """
        Construct and save a binvox-formatted voxel grid of environment
        Args:
            position (Vector3r): Position around which voxel grid is centered in m
            x, y, z (int): Size of each voxel grid dimension in m
            res (float): Resolution of voxel grid in m
            of (str): Name of output file to save voxel grid as
        Returns:
            bool: True if output written to file successfully, else False
        """
        return self.client.simCreateVoxelGrid(position, x, y, z, res, of)

    # Add new vehicle via RPC
    def simAddVehicle(self, vehicle_name, vehicle_type, pose, pawn_path=""):
        """
        Create vehicle at runtime
        Args:
            vehicle_name (str): Name of the vehicle being created
            vehicle_type (str): Type of vehicle, e.g. "simpleflight"
            pose (Pose): Initial pose of the vehicle
            pawn_path (str, optional): Vehicle blueprint path, default empty wbich uses the default blueprint for the vehicle type
        Returns:
            bool: Whether vehicle was created
        """
        return self.client.simAddVehicle(vehicle_name, vehicle_type, pose, pawn_path)

    def listVehicles(self):
        """
        Lists the names of current vehicles
        Returns:
            list[str]: List containing names of all vehicles
        """

        return self.assigned_blueprint

    def getSettingsString(self):
        """
        Fetch the settings text being used by AirSim
        Returns:
            str: Settings text in JSON format
        """
        a = [self.client.getSettingsString()]
        for each in self.airsim_client_list:
            a.append(each.getSettingsString())
        return a

    def takeoffAsync(self, timeout_sec=20, vehicle_name=''):
        """
        Takeoff vehicle to 3m above ground. Vehicle should not be moving when this API is used
        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.takeoffAsynck(timeout_sec, self.vehicle_dict[vehicle_name].airsim_name)

    def landAsync(self, timeout_sec=60, vehicle_name=''):
        """
        Land the vehicle
        Args:
            timeout_sec (int, optional): Timeout for the vehicle to land
            vehicle_name (str, optional): Name of the vehicle to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.landAsync(
            timeout_sec, self.vehicle_dict[vehicle_name].airsim_name)

    def goHomeAsync(self, timeout_sec=3e+38, vehicle_name=''):
        """
        Return vehicle to Home i.e. Launch location
        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.goHomeAsync(
            timeout_sec, self.vehicle_dict[vehicle_name].airsim_name)

    # APIs for control
    def moveByVelocityBodyFrameAsync(self, vx, vy, vz, duration, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                                     yaw_mode=YawMode(), vehicle_name=''):
        """
        Args:
            vx (float): desired velocity in the X axis of the vehicle's local NED frame.
            vy (float): desired velocity in the Y axis of the vehicle's local NED frame.
            vz (float): desired velocity in the Z axis of the vehicle's local NED frame.
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByVelocityBodyFrameAsync(
            vx, vy, vz, duration, drivetrain, yaw_mode, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByVelocityZBodyFrameAsync(self, vx, vy, z, duration, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                                      yaw_mode=YawMode(), vehicle_name=''):
        """
        Args:
            vx (float): desired velocity in the X axis of the vehicle's local NED frame
            vy (float): desired velocity in the Y axis of the vehicle's local NED frame
            z (float): desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """

        return self.vehicle_dict[vehicle_name].client.moveByVelocityZBodyFrameAsync(
            vx, vy, z, duration, drivetrain, yaw_mode,
            self.vehicle_dict[vehicle_name].airsim_name)

    def moveByAngleZAsync(self, pitch, roll, z, yaw, duration, vehicle_name=''):
        logging.warning("moveByAngleZAsync API is deprecated, use moveByRollPitchYawZAsync() API instead")
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawZ(pitch, roll, z,  yaw, duration,
                                                                          self.vehicle_dict[vehicle_name].airsim_name)

    def moveByAngleThrottleAsync(self, pitch, roll, throttle, yaw_rate, duration, vehicle_name=''):
        logging.warning(
            "moveByAngleThrottleAsync API is deprecated, use moveByRollPitchYawrateThrottleAsync() API instead")
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawrateThrottle(
            roll, pitch, yaw_rate, throttle, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                            yaw_mode=YawMode(), vehicle_name=''):
        """
        Args:
            vx (float): desired velocity in world (NED) X axis
            vy (float): desired velocity in world (NED) Y axis
            vz (float): desired velocity in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByVelocity(
            vx, vy, vz, duration, drivetrain, yaw_mode, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                             yaw_mode=YawMode(), vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.moveByVelocityZAsync(
            vx, vy, z, duration, drivetrain, yaw_mode, self.vehicle_dict[vehicle_name].airsim_name)

    def moveOnPathAsync(self, path, velocity, timeout_sec=3e+38, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                        yaw_mode=YawMode(),
                        lookahead=-1, adaptive_lookahead=1, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.moveOnPathAsync(
            path, velocity, timeout_sec, drivetrain, yaw_mode,
            lookahead, adaptive_lookahead, self.vehicle_dict[vehicle_name].airsim_name)

    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec=3e+38, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                            yaw_mode=YawMode(),
                            lookahead=-1, adaptive_lookahead=1, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.moveToPositionAsync(
                    x, y, z, velocity, timeout_sec, drivetrain, yaw_mode, lookahead,
                    adaptive_lookahead, self.vehicle_dict[vehicle_name].airsim_name)

    def moveToGPSAsync(self, latitude, longitude, altitude, velocity, timeout_sec=3e+38,
                       drivetrain=DrivetrainType.MaxDegreeOfFreedom, yaw_mode=YawMode(),
                       lookahead=-1, adaptive_lookahead=1, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.moveToGPSAsync(latitude, longitude,
                                                              altitude, velocity, timeout_sec, drivetrain, yaw_mode,
                                                              lookahead, adaptive_lookahead,
                                                              self.vehicle_dict[vehicle_name].airsim_name)

    def moveToZAsync(self, z, velocity, timeout_sec=3e+38, yaw_mode=YawMode(), lookahead=-1, adaptive_lookahead=1,
                     vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.moveToZAsync(
            z, velocity, timeout_sec, yaw_mode, lookahead, adaptive_lookahead,
            self.vehicle_dict[vehicle_name].airsim_name)

    def moveByManualAsync(self, vx_max, vy_max, z_min, duration, drivetrain=DrivetrainType.MaxDegreeOfFreedom,
                          yaw_mode=YawMode(), vehicle_name=''):
        """
        - Read current RC state and use it to control the vehicles.
        Parameters sets up the constraints on velocity and minimum altitude while flying. If RC state is detected to violate these constraints
        then that RC state would be ignored.
        Args:
            vx_max (float): max velocity allowed in x direction
            vy_max (float): max velocity allowed in y direction
            vz_max (float): max velocity allowed in z direction
            z_min (float): min z allowed for vehicle position
            duration (float): after this duration vehicle would switch back to non-manual mode
            drivetrain (DrivetrainType): when ForwardOnly, vehicle rotates itself so that its front is always facing the direction of travel. If MaxDegreeOfFreedom then it doesn't do that (crab-like movement)
            yaw_mode (YawMode): Specifies if vehicle should face at given angle (is_rate=False) or should be rotating around its axis at given rate (is_rate=True)
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByManualAsync(
            vx_max, vy_max, z_min, duration, drivetrain, yaw_mode,
            self.vehicle_dict[vehicle_name].airsim_name)

    def rotateToYawAsync(self, yaw, timeout_sec=3e+38, margin=5, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.rotateToYawAsync(
            yaw, timeout_sec, margin,  self.vehicle_dict[vehicle_name].airsim_name)

    def rotateByYawRateAsync(self, yaw_rate, duration, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.rotateByYawRateAsync(
            yaw_rate, duration,  self.vehicle_dict[vehicle_name].airsim_name)

    def hoverAsync(self, vehicle_name=''):
        return self.vehicle_dict[vehicle_name].client.hoverAsync(self.vehicle_dict[vehicle_name].airsim_name)

    def moveByRC(self, rcdata=RCData(), vehicle_name=''):
        return self.moveByRC(rcdata, self.vehicle_dict[vehicle_name].airsim_name)

    # low - level control API
    def moveByMotorPWMsAsync(self, front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration,
                             vehicle_name=''):
        """
        - Directly control the motors using PWM values
        Args:
            front_right_pwm (float): PWM value for the front right motor (between 0.0 to 1.0)
            rear_left_pwm (float): PWM value for the rear left motor (between 0.0 to 1.0)
            front_left_pwm (float): PWM value for the front left motor (between 0.0 to 1.0)
            rear_right_pwm (float): PWM value for the rear right motor (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByMotorPWMsAsync(
            front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm,
            duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByRollPitchYawZAsync(self, roll, pitch, yaw, z, duration, vehicle_name=''):
        """
        - z is given in local NED frame of the vehicle.
        - Roll angle, pitch angle, and yaw angle set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw (float): Desired yaw angle, in radians.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawZAsync(
            roll, pitch, yaw, z, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByRollPitchYawThrottleAsync(self, roll, pitch, yaw, throttle, duration, vehicle_name=''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw angle are given in **degrees** when using PX4 and in **radians** when using SimpleFlight, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll (float): Desired roll angle.
            pitch (float): Desired pitch angle.
            yaw (float): Desired yaw angle.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByRollPitchYawrateThrottleAsync(self, roll, pitch, yaw_rate, throttle, duration, vehicle_name=''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawrateThrottleAsync(
            roll, pitch, yaw_rate, throttle, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByRollPitchYawrateZAsync(self, roll, pitch, yaw_rate, z, duration, vehicle_name=''):
        """
        - z is given in local NED frame of the vehicle.
        - Roll angle, pitch angle, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll (float): Desired roll angle, in radians.
            pitch (float): Desired pitch angle, in radians.
            yaw_rate (float): Desired yaw rate, in radian per second.
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByRollPitchYawrateZAsync(
            roll, pitch, yaw_rate, z, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByAngleRatesZAsync(self, roll_rate, pitch_rate, yaw_rate, z, duration, vehicle_name=''):
        """
        - z is given in local NED frame of the vehicle.
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            z (float): Desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByAngleRatesZAsync(
            roll_rate, pitch_rate, yaw_rate, z, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def moveByAngleRatesThrottleAsync(self, roll_rate, pitch_rate, yaw_rate, throttle, duration, vehicle_name=''):
        """
        - Desired throttle is between 0.0 to 1.0
        - Roll rate, pitch rate, and yaw rate set points are given in **radians**, in the body frame.
        - The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        - Frame Convention:
            - X axis is along the **Front** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **roll** angle.
            | Hence, rolling with a positive angle is equivalent to translating in the **right** direction, w.r.t. our FLU body frame.
            - Y axis is along the **Left** direction of the quadrotor.
            | Clockwise rotation about this axis defines a positive **pitch** angle.
            | Hence, pitching with a positive angle is equivalent to translating in the **front** direction, w.r.t. our FLU body frame.
            - Z axis is along the **Up** direction.
            | Clockwise rotation about this axis defines a positive **yaw** angle.
            | Hence, yawing with a positive angle is equivalent to rotated towards the **left** direction wrt our FLU body frame. Or in an anticlockwise fashion in the body XY / FL plane.
        Args:
            roll_rate (float): Desired roll rate, in radians / second
            pitch_rate (float): Desired pitch rate, in radians / second
            yaw_rate (float): Desired yaw rate, in radians / second
            throttle (float): Desired throttle (between 0.0 to 1.0)
            duration (float): Desired amount of time (seconds), to send this command for
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.vehicle_dict[vehicle_name].client.moveByAngleRatesThrottleAsync(
            roll_rate, pitch_rate, yaw_rate, throttle, duration, self.vehicle_dict[vehicle_name].airsim_name)

    def setAngleRateControllerGains(self, angle_rate_gains=AngleRateControllerGains(), vehicle_name=''):
        """
        - Modifying these gains will have an affect on *ALL* move*() APIs.
            This is because any velocity setpoint is converted to an angle level setpoint which is tracked with an angle level controllers.
            That angle level setpoint is itself tracked with and angle rate controller.
        - This function should only be called if the default angle rate control PID gains need to be modified.
        Args:
            angle_rate_gains (AngleRateControllerGains):
                - Correspond to the roll, pitch, yaw axes, defined in the body frame.
                - Pass AngleRateControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.vehicle_dict[vehicle_name].client.setAngleRateControllerGains(
            angle_rate_gains, self.vehicle_dict[vehicle_name].airsim_name)

    def setAngleLevelControllerGains(self, angle_level_gains=AngleLevelControllerGains(), vehicle_name=''):
        """
        - Sets angle level controller gains (used by any API setting angle references - for ex: moveByRollPitchYawZAsync(), moveByRollPitchYawThrottleAsync(), etc)
        - Modifying these gains will also affect the behaviour of moveByVelocityAsync() API.
            This is because the AirSim flight controller will track velocity setpoints by converting them to angle set points.
        - This function should only be called if the default angle level control PID gains need to be modified.
        - Passing AngleLevelControllerGains() sets gains to default airsim values.
        Args:
            angle_level_gains (AngleLevelControllerGains):
                - Correspond to the roll, pitch, yaw axes, defined in the body frame.
                - Pass AngleLevelControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.vehicle_dict[vehicle_name].client.setAngleLevelControllerGains(
            angle_level_gains, self.vehicle_dict[vehicle_name].airsim_name)

    def setVelocityControllerGains(self, velocity_gains=VelocityControllerGains(), vehicle_name=''):
        """
        - Sets velocity controller gains for moveByVelocityAsync().
        - This function should only be called if the default velocity control PID gains need to be modified.
        - Passing VelocityControllerGains() sets gains to default airsim values.
        Args:
            velocity_gains (VelocityControllerGains):
                - Correspond to the world X, Y, Z axes.
                - Pass VelocityControllerGains() to reset gains to default recommended values.
                - Modifying velocity controller gains will have an affect on the behaviour of moveOnSplineAsync() and moveOnSplineVelConstraintsAsync(), as they both use velocity control to track the trajectory.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.vehicle_dict[vehicle_name].client.setVelocityControllerGains(velocity_gains, self.vehicle_dict[vehicle_name].airsim_name)

    def setPositionControllerGains(self, position_gains=PositionControllerGains(), vehicle_name=''):
        """
        Sets position controller gains for moveByPositionAsync.
        This function should only be called if the default position control PID gains need to be modified.
        Args:
            position_gains (PositionControllerGains):
                - Correspond to the X, Y, Z axes.
                - Pass PositionControllerGains() to reset gains to default recommended values.
            vehicle_name (str, optional): Name of the multirotor to send this command to
        """
        self.vehicle_dict[vehicle_name].client.setPositionControllerGains(position_gains, self.vehicle_dict[vehicle_name].airsim_name)

    # query vehicle state
    def getMultirotorState(self, vehicle_name=''):
        """
        The position inside the returned MultirotorState is in the frame of the vehicle's starting point
        Args:
            vehicle_name (str, optional): Vehicle to get the state of
        Returns:
            MultirotorState:
        """
        return self.vehicle_dict[vehicle_name].client.getMultirotorState(self.vehicle_dict[vehicle_name].airsim_name)

    # query rotor states
    def getRotorStates(self, vehicle_name=''):
        """
        Used to obtain the current state of all a multirotor's rotors. The state includes the speeds,
        thrusts and torques for all rotors.
        Args:
            vehicle_name (str, optional): Vehicle to get the rotor state of
        Returns:
            RotorStates: Containing a timestamp and the speed, thrust and torque of all rotors.
        """
        return self.vehicle_dict[vehicle_name].client.getRotorStates(self.vehicle_dict[vehicle_name].airsim_name)


if __name__ == '__main__':
    s = socket(AF_INET, SOCK_STREAM)
    s.bind(('127.0.0.1', 9699))
    s.listen(200)  # 最大连接数
    client = CustomAirsimClient(["172.18.0.2", "172.18.0.2"], s)
    dic = {'1': [1, 0],
           '2': [1, 1],
           '3': [0, 1],
           '4': [0, 0]}
    k = 0
    time.sleep(2)
    while True:
        # client.reset()
        a = client.listVehicles()
        x, y = dic[str(k+1)]
        lista = []
        for name in a:
            client.moveToPositionAsync(x, y, -3, 2, vehicle_name=name)
            # lista.append(client.moveToPositionAsync(
            #     x+k-10*int(client.vehicle_dict[name].client.ip[-1]), 0, -3, 2, vehicle_name=name))
        for each in lista:
            each.join()

        k += 1
        k %= 4
    # c1.run()

    # c1.run()
    # c2.run()
