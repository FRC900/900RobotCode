#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu

def rezero_accelerator(req):
    global accelerometer
    x = accelerometer.raw_x
    y = accelerometer.raw_y
    z = accelerometer.raw_z

    accelerometer.offset = (
    round(-x / 4),
    round(-y / 4),
    round(-(z - 20) / 4))  # Z should be '20' at 1g (49mg per bit)  

pub = rospy.Publisher("vroom",Imu,queue_size=1)
s = rospy.Service('Zane', Empty, rezero_accelerator)

rospy.init_node("vroom", anonymous=True)

import board
import adafruit_adxl37x
from adafruit_adxl34x import DataRate

i2c = board.I2C()  # uses board.SCL and board.SDA
global accelerometer
accelerometer = adafruit_adxl37x.ADXL375(i2c)
accelerometer.data_rate  = DataRate.RATE_800_HZ

_REG_DATA_FORMAT: int = 0x31

#_INT_ACT: int = 0b00010000

#so this maybe?
active_change = accelerometer._read_register_unpacked(_REG_DATA_FORMAT) 
#accelerometer._write_resgiter_byte(_REG_INT_ENABLE, 0x0) #disables interrupts, so that we can write new things to registers without messing other things up?
#accelerometer._write_register_byte(_REG_DATA_FORMAT, 0b00001011) #0b00001011 comes from the adafruit arduino library linked by ty which supposedly fixes the bits at _reg_data_format for 25+ g
#accelerometer._write_register_byte(_REG_DATA_FORMAT, _INT_ACT) # ACT bit
#active_change |= _INT_ACT
accelerometer._write_register_byte(_REG_DATA_FORMAT, active_change | 0b00001011) #0b00001011 comes from the adafruit arduino library linked by ty which supposedly fixes the bits at _reg_data_format for 25+ g

r = rospy.Rate(800)

while not rospy.is_shutdown():
    message=Imu()
    message.header.stamp = rospy.get_rostime()
    x,y,z = accelerometer.acceleration
    message.linear_acceleration.x = x
    message.linear_acceleration.y = y
    message.linear_acceleration.z = z
    pub.publish(message)
    r.sleep()
