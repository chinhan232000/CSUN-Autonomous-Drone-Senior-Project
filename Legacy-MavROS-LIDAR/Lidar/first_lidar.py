# rplidar package from Roboticia works for RPLIDAR firmware 1.29 which is currently in use
from rplidar import RPLidar

print("Starting lidar data collection program...")
lidar = RPLidar('/dev/ttyUSB0')
lidar.reset()

health = lidar.get_health()
print(health)
info = lidar.get_info()
print(info)

for i, scan in enumerate(lidar.iter_scans()):
    print(scan)
    if i > 100:
        break

lidar.disconnect()
