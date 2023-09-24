#!/usr/bin/env python3

import sys
import asyncio          # Asyncio allows for near concurrent processing 
import read_pi_pico     # Lidar connection using rpi pico GPIO pinout
    # Mavsdk interacts with the drone through serial port USB0/1 or ACM0/1 
    # Mavsdk.offboard allows manual altitude/thrust/tilt/GPS control over drone
from mavsdk.geofence import Point, Polygon # Perimeter for drone
from mavsdk.offboard import (Attitude, OffboardError) # Altitude Control
from mavsdk.offboard import (OffboardError, PositionNedYaw) # GPS Control
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed) # Velocity Control
from mavsdk.camera import (CameraError, Mode)

class util():
    def __init__(self):
        self.abs = 0.0
        self.heading = 0.0
        self.lidar_distance = 0
        

    async def setup_info(self, drone):   # , LIDAR
        # Start parallel tasks that will be awaiting changes in the drone and track them
        self.print_mission_progress_task = asyncio.ensure_future(self.print_mission_progress(drone))
        self.print_altitude_task = asyncio.ensure_future(self.print_altitude(drone))
        self.print_flight_mode_task = asyncio.ensure_future(self.print_flight_mode(drone))

        # Used to check that the drone is flying, monitors altitude & flight mode. Termination task raised if drone lands
        self.running_tasks = [self.print_altitude_task, self.print_flight_mode_task]
        self.termination_task = asyncio.ensure_future(self.observe_is_in_air(drone, self.running_tasks))

        # Future variables that will be updated for px4flow camera use
        self.print_mode_task = asyncio.ensure_future(self.print_mode(drone))
        self.print_status_task = asyncio.ensure_future(self.print_status(drone))
        self.print_tasks = [self.print_mode_task, self.print_status_task]

        # Shell for connecting to px4
        asyncio.ensure_future(self.observe_shell(drone))

        # LIDAR Distance saved from a future LIDAR class
        # self.lidar_distance = asyncio.ensure_future(   LIDAR.distance   )


    # Connect to the drone by serial / tcp / udp, verify connection, start a log file
    async def start_connection(self, drone, sim):
        try:
            if ( (sim == None) or (sim == '0')):
                ## Connect to drone using usb-to-microusb over serial port on pixhawk 4 
                # await drone.connect(system_address="serial:///dev/ttyACM0:921600") # - Or /dev/ttyUSB
                # Connect using ./mavsdk_server hosted over 'localhost' port:50051
                await drone.connect()               

            elif (sim == '1'):      # Connect to drone using simulator if '-s 1' given as argument
                await drone.connect(system_address="udp://:14540")

        except asyncio.TimeoutError:
            print(sys.stderr)
            return 

        # # Awaiting the next task's status
        self.status_text_task = asyncio.ensure_future(self.print_status_text(drone))

        # Will halt script until connection made
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        # # Start log file to be saved in current directory
        # entries = await self.get_entries(drone)
        # for entry in entries:
        #     await self.download_log(drone, entry)


    # Called when the drone needs to check connection / telemetry / health           
    async def check_status(self, drone):
        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break


    # Print the values of the telemetry status and throw exception for errors
    async def print_status_text(self, drone):
        try:
            async for status_text in drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return


    # Initializes the drone calibration sensors
    async def calibrate_drone(self, drone):
        print("-- Starting gyroscope calibration")
        async for progress_data in drone.calibration.calibrate_gyro():
            print(progress_data)
        print("-- Gyroscope calibration finished")

        print("-- Starting accelerometer calibration")
        async for progress_data in drone.calibration.calibrate_accelerometer():
            print(progress_data)
        print("-- Accelerometer calibration finished")

        print("-- Starting magnetometer calibration")
        async for progress_data in drone.calibration.calibrate_magnetometer():
            print(progress_data)
        print("-- Magnetometer calibration finished")

        print("-- Starting board level horizon calibration")
        async for progress_data in drone.calibration.calibrate_level_horizon():
            print(progress_data)
        print("-- Board level calibration finished")


    # Lands drone and cancels next task status
    async def land_drone(self, drone, util):
        print("-- Landing")
        await drone.action.land()
        print(util.termination_task)
        # self.status_text_task.cancel()


    # Arms and prints to terminal
    async def arm_drone(self, drone):
        print("-- Arming")
        await drone.action.arm()
        

    # Starts offboard controls for drone movement
    async def start_offboard(self, drone):
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return


    # Ends offboard controls for drone movement
    async def stop_offboard(self, drone):
        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: \
                {error._result.result}")
            

    # Starts offboard controls for drone movement
    async def download_log(self, drone, entry):
        date_without_colon = entry.date.replace(":", "-")
        filename = f"./log-{date_without_colon}.ulog"
        print(f"Downloading: log {entry.id} from {entry.date} to {filename}")
        previous_progress = -1
        async for progress in drone.log_files.download_log_file(entry, filename):
            new_progress = round(progress.progress*100)
            if new_progress != previous_progress:
                sys.stdout.write(f"\r{new_progress} %")
                sys.stdout.flush()
                previous_progress = new_progress
        # print()


    # Prints entries in the log file
    async def get_entries(self, drone):
        entries = await drone.log_files.get_entries()
        for entry in entries:
            print(f"Log {entry.id} from {entry.date}")
        return entries
    

    # Find direction drone is facing
    async def get_heading(self, drone):
        print("Reading Heading (Yaw 0->360)...")
        async for heading_deg in drone.telemetry.heading():
            self.heading = heading_deg.heading_deg
            print("-- Current heading in degrees (0 North, positive is clock-wise looking from above): {}", self.heading)
            break


    # Calculates telemetry absolute altitude
    async def abs_altitude(self, drone):
        print("Fetching coordinates & amsl altitude at home location....")
        async for terrain_info in drone.telemetry.home():
            self.absolute_altitude = terrain_info.absolute_altitude_m
            self.latitude = terrain_info.latitude_deg
            self.longitude = terrain_info.longitude_deg
            break


    # Prints the altitude when it changes
    async def print_altitude(self, drone):

        previous_altitude = None

        async for position in drone.telemetry.position():
            altitude = round(position.relative_altitude_m)
            if altitude != previous_altitude:
                previous_altitude = altitude
                print(f"Altitude: {altitude}")


    # Prints the flight mode when it changes 
    async def print_flight_mode(self, drone):
        previous_flight_mode = None

        async for flight_mode in drone.telemetry.flight_mode():
            if flight_mode != previous_flight_mode:
                previous_flight_mode = flight_mode
                print(f"Flight mode: {flight_mode}")


    # Monitors whether the drone is flying or not and returns after landing
    async def observe_is_in_air(self, drone, util):
        was_in_air = False

        async for is_in_air in drone.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                print("--landed--")


    # Prints the progress of the mission according to plan
    async def print_mission_progress(self, drone):
        async for mission_progress in drone.mission.mission_progress():
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")
        

    async def define_geofence(self, drone):    
        await asyncio.sleep(0.5)

        # Define your geofence boundary
        # p1 = Point(self.latitude - 0.0001, self.longitude - 0.0001)
        # p2 = Point(self.latitude + 0.0001, self.longitude - 0.0001)
        # p3 = Point(self.latitude + 0.0001, self.longitude + 0.0001)
        # p4 = Point(self.latitude - 0.0001, self.longitude + 0.0001)

        p2 = Point(34.2424481, -118.5244912); p3 = Point(34.2424514, -118.5234616)
        p1 = Point(34.2411322, -118.5244574); p4 = Point(34.2411757, -118.5234774)

        # Create a polygon object using your points
        polygon = Polygon([p1, p2, p3, p4], Polygon.FenceType.INCLUSION)

        # Upload the geofence to your vehicle
        print("Uploading geofence...")
        await drone.geofence.upload_geofence([polygon])

        print("Geofence uploaded!")


    # Get the list of parameters then iterate through all int parameters
    async def print_all_params(self, drone):
        all_params = await drone.param.get_all_params()

        for param in all_params.int_params:
            print(f"{param.name}: {param.value}")

        for param in all_params.float_params:
            print(f"{param.name}: {param.value}")


    # Prints the flight mode when it changes
    async def print_flight_mode(self, drone):

        previous_flight_mode = None

        async for flight_mode in drone.telemetry.flight_mode():
            if flight_mode != previous_flight_mode:
                previous_flight_mode = flight_mode
                print(f"Flight mode: {flight_mode}")


    async def use_camera(self, drone):

        print("Setting mode to 'PHOTO'")
        try:
            await drone.camera.set_mode(Mode.PHOTO)
        except CameraError as error:
            print(f"Setting mode failed with error code: {error._result.result}")

        await asyncio.sleep(2)

        # async for setting in drone.camera.get_setting():
        #     print(setting)
        
        # f = await drone.camera.list_photos(0)

        # await print(f)

        async for stat in drone.camera.status():
            print(f"Status: {stat}")

        await asyncio.sleep(2)

        print("Taking a photo")
        try:
            await drone.camera.take_photo()
        except CameraError as error:
            print(f"Couldn't take photo: {error._result.result}")


    async def print_mode(self, drone):
        async for mode in drone.camera.mode():
            print(f"Camera mode: {mode}")


    async def print_status(self, drone):
        async for status in drone.camera.status():
            print(status)


    # Next 4 programs used to connect to px4 shell over serial connection   #   #  #
    async def connect_shell(self,drone):                                           #
        asyncio.get_event_loop().add_reader(sys.stdin, self.got_stdin_data, drone) #
        print("nsh> ", end='', flush=True)                                         #
    #                                                                              #
    async def observe_shell(self, drone):                                          #
        async for output in drone.shell.receive():                                 #
            print(f"\n{output} ", end='', flush=True)                              #
    #                                                                              #
    def got_stdin_data(self, drone):                                               #
        asyncio.ensure_future(self.send(drone, sys.stdin.readline()))              #
    #                                                                              #
    async def send(self, drone, command):                                          #
        await drone.shell.send(command)                                            #
    #   #   #   #   #   #   #   #   #   #   #   #   #   #   #   #   #   #   #   #  #


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#                       TO BE ADDED 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#     # Start the tasks
#     asyncio.ensure_future(print_battery(drone))
#     asyncio.ensure_future(print_gps_info(drone))
#     asyncio.ensure_future(print_in_air(drone))
#     asyncio.ensure_future(print_position(drone))

# async def print_battery(drone):
#     async for battery in drone.telemetry.battery():
#         print(f"Battery: {battery.remaining_percent}")

# async def print_gps_info(drone):
#     async for gps_info in drone.telemetry.gps_info():
#         print(f"GPS info: {gps_info}")

# async def print_in_air(drone):
#     async for in_air in drone.telemetry.in_air():
#         print(f"In air: {in_air}")

# async def print_position(drone):
#     async for position in drone.telemetry.position():
#         print(position)

# async for flight_mode in drone.telemetry.flight_mode():
#     print("FlightMode:", flight_mode)