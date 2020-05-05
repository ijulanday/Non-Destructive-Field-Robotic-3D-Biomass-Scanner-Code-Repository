from rplidar import RPLidar
import os
import time

# lidar -> lidar unit object; data -> array of data
def TakeScan(lidar, data):
    # scan -> a list of tuples (quality, angle, distance) in one rotation
    for i, scan in enumerate(lidar.iter_scans()):
        print('%d: Got %d measurements' % (i, len(scan)))
        data.append(scan)
        # arbitrary revolution limit
        if i>10:
            break
    # PWM needs to be low enough to limit current draw (stay under ~3A USB limit)
    lidar.set_pwm(400)
    # also not necessary
    lidar.stop()
        
data = []

lidars = []
numlidars = 3

print('powering up LiDARs...')
for i in range(0,numlidars):
    lidars.append(RPLidar('/dev/ttyUSB' + str(i)))
    print(lidars[i].get_info())
    print(lidars[i].get_health())
    lidars[i].set_pwm(400)
    time.sleep(3)

time.sleep(.1)

print('letting LiDARs run for a bit')
for i in range(0,5):
    print('.')
    time.sleep(1)
    
time.sleep(.1)


# collect lidar data
i = 0
for lidar in lidars:
    print('getting data from lidar '+ str(i))
    TakeScan(lidar, data)
time.sleep(1)
    
for i in range(0,5):
    print('.')
    time.sleep(1)

print('shutting down units...')
for lidar in lidars:
    lidar.stop_motor()
    time.sleep(0.2)
    lidar.stop()
    time.sleep(0.2)
    lidar.disconnect()
    time.sleep(2.5)
    
print('writing data to file...')
f = open('rplidar_testbench_output.txt', 'w')
for chunk in data:
  for scan in chunk:
    f.write(str(scan[1]) + ',' + str(scan[2]) + '\n')
f.close()

print('\n\ntestbench complete!')
