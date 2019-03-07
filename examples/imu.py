import board
import busio
import adafruit_mpu9250

i2c = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_mpu9250.MPU9250_I2C(i2c)

print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
print('Temperature: {0:0.3f}C'.format(sensor.temperature))

