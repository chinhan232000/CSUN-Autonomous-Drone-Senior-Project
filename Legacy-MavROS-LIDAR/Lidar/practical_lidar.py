import multiprocessing as mp
import numpy as np
import time
from rplidar import RPLidar

def lidar_data(quality_array, angle_array, distance_array):

    print("Starting lidar data collection program...")
    lidar = RPLidar('/dev/ttyUSB0')
    lidar.reset()

    health = lidar.get_health()
    print(health)
    info = lidar.get_info()
    print(info)

    for scan in lidar.iter_scans():
        quality = []
        angle = []
        distance = []

        # This is necessary to reset the shared array since not all data is same array lengths and can linger in other arrays
        # ^ In otherwords there was no guarantee that new array would completely overwrite the prior array
        # ^ However, a "ready for pickup" variable should be used so that NAN arrays arn't being read
        for index in range(len(quality_array)):
            quality_array[index] = np.nan
            angle_array[index] = np.nan
            distance_array[index] = np.nan

        print("start of data chunk")
        for data in scan:
            print(f"data: {data}")
            
            quality.append(data[0])
            angle.append(data[1])
            distance.append(data[2])
        print("end of data chunk")
        print(f"size of chunk: {len(quality)}")

        print(f"quality: {quality}")
        print(f"angle: {angle}")
        print(f"distance: {distance}")

        # The below element wise method is what works for sending the data outside of this process
        for index, x in enumerate(quality):
            quality_array[index] = x
        for index, y in enumerate(angle):
            angle_array[index] = y
        for index, z in enumerate(distance):
            distance_array[index] = z
            
if __name__ == '__main__':
    # If issues arise with filling shared array consider that there may be not enough room and to increase ARRAY_LENGTH (increased from 150 to 750)
    ARRAY_LENGTH = 750
    quality_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)
    angle_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)
    distance_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)

    p = mp.Process(target=lidar_data, args=(quality_array, angle_array, distance_array))
    p.start()

    while True:
        print(f"received quality: {quality_array[:]}")
        print(f"received angle: {angle_array[:]}")
        print(f"received distance: {distance_array[:]}")
        time.sleep(0.3)
