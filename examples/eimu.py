import time
from mpu9250 import MPU9250

sensor = MPU9250()

print("MPU9250 id: " + hex(sensor.whoami))

while True:
	print(sensor.acceleration)
	print(sensor.gyro)
	print(sensor.magnetic)
	time.sleep(1000)
