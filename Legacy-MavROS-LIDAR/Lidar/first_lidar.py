# rplidar package from Roboticia works for RPLIDAR firmware 1.29 which is currently in use
from rplidar import RPLidar

# Function to collect lidar scan data
print("Starting lidar data collection program...")
# Create an instance of the RPLidar class and specify the USB port ('/dev/ttyUSB0') where the lidar is connected
lidar = RPLidar('/dev/ttyUSB0')
# Reset the lidar device
lidar.reset()

# Get health information from the lidar device and store it in the 'health' variable
health = lidar.get_health()
print(health)
info = lidar.get_info()
print(info)

# Iterate over lidar scans
for i, scan in enumerate(lidar.iter_scans()):
 # Print the lidar scan data for each iteration
    print(scan)

    # Exit the loop if more than 100 iterations have been completed (optional)
    if i > 100:
        break
 # Disconnect from the lidar device when done
lidar.disconnect()
