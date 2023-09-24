import multiprocessing as mp
import numpy as np
import time
from rplidar import RPLidar

def lidar_data(quality_array, angle_array, distance_array, upating_data):

    print("Starting Lidar...")
    lidar = RPLidar('/dev/ttyUSB0')
    lidar.reset()

    health = lidar.get_health()
    print(health)
    info = lidar.get_info()
    print(info)

    for scan in lidar.iter_scans():
        # Local variables used to accumulate the lidar data before any possible transmission through shared memory 
        quality = []
        angle = []
        distance = []

        # Portion where lidar is free to run wild and generate data that accumulates inside the prior local variables
        #print("Start of Data Chunk")
        for data in scan:
            #print(f"data: {data}")
            
            quality.append(data[0])
            angle.append(data[1])
            distance.append(data[2])
        #print("End of Data Chunk")
        #print(f"Size of Chunk: {len(quality)}")

        #print(f"quality: {quality}")
        #print(f"angle: {angle}")
        #print(f"distance: {distance}")

        print(f"LIDAR: updating_data.value: {updating_data.value}")
        if updating_data.value == True:
            # Shared arrays have their data cleared with nan values (could be more efficient by stopping when a nan value is first encountered)
            for index in range(len(quality_array)):
                quality_array[index] = np.nan
                angle_array[index] = np.nan
                distance_array[index] = np.nan

            # Shared arrays are filled with the data acculuated into the local variables intended to hold a chunk of data from the lidar
            for index, value in enumerate(quality):
                quality_array[index] = value
            for index, value in enumerate(angle):
                angle_array[index] = value
            for index, value in enumerate(distance):
                distance_array[index] = value

            updating_data.value = False
            print("LIDAR: Sent Data Chunk")
 
if __name__ == '__main__':
    # If issues arise with filling shared array consider that there may be not enough room and to increase ARRAY_LENGTH (increased from 150 to 750)
    ARRAY_LENGTH = 750
    quality_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)
    angle_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)
    distance_array = mp.Array('f', np.ones(ARRAY_LENGTH)*np.nan)
    updating_data = mp.Value('I', False)

    print(f"INIT: updating_data: {updating_data.value}")
    
    lidar_process = mp.Process(target = lidar_data, args = (quality_array, angle_array, distance_array, updating_data))
    lidar_process.start()

    # [TODO] need to implement below code with asyncio 
    while True:
        print(f"CURRENT: updating_data: {updating_data.value}")
        updating_data.value = True
        print(f"Set updating_data.value to 1")
        while updating_data.value == True:
            print("Waiting for Update to Complete")
            # This time.sleep is to prevent the line from being too "hot" doing nothing (with asyncio with would be the asyncio.sleep(0.1))
            time.sleep(0.1)
        print("Update Completed")
        
        # Note: not shown but matplotlib was used to visualize the data and then code removed (just as a sanity check of the data accuracy)
        #print(f"received quality: {quality_array[:]}")
        print(f"received angle: {angle_array[:]}")
        print(f"received distance: {distance_array[:]}")

        # This time.sleep is to simulate the occasional request of data from the lidar (intentionally much slower than the lidar's speed)
        time.sleep(1)