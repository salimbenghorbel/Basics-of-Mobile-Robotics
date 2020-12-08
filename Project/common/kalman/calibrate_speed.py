distance = 29.7/100
time = 26.3/3
sensor_v = 100

velocity = distance/time
speed_conv_factor = velocity/sensor_v
print(speed_conv_factor)
print('velocity (m/s) = sensor * ' + str(speed_conv_factor))
