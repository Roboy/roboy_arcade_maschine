# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685
import rospy
from roboy_communication_middleware.msg import MotorCommand

servo_min = 145  # Min pulse length out of 4096
servo_max = 560  # Max pulse length out of 4096

def callback(data):
    global servo_min
    global servo_max
#    rospy.loginfo("revceived motor command")
#    rospy.loginfo(data.motors)
    #rospy.loginfo(int(data.motors[0]))    
    for (motor,setpoint) in zip(data.motors,data.setPoints):
    	pwm.set_pwm(motor, 0, int(setpoint/180.0*(servo_max-servo_min+servo_min)))
#	rospy.loginfo(str(motor))
#	rospy.loginfo(str(motor) + " " + str(setpoint))


# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths

# Main function.
if __name__ == '__main__':
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()
    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(60)
    # Initialize the node and name it.
    rospy.init_node('roboy_arcade_maschine')
    rospy.Subscriber("roboy/communication/middleware", MotorCommand, callback)

    rospy.spin()
