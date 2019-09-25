from mpu9250 import MPU9250, MPU6500, AK8963
from time import sleep

mpu = MPU6500(address=0x69)
ak = AK8963()

#offset, scale = ak.calibrate(count=256, delay=200)

sensor = MPU9250(mpu, ak)

print("Reading in data from IMU")
print("MPU9250 id: " + hex(sensor.read_whoami()))

while True:
    print("")
    print("Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(*sensor.read_acceleration()))
    print("Magnetometer (gauss): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(*sensor.read_magnetic()))
    print("Gyroscope (degrees/sec): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(*sensor.read_acceleration()))
    print("Temperature: {0:0.3f}C".format(sensor.mpu6500.read_temperature()))
    sleep(2)
