#!/usr/bin/env python3

"""
Crobot Motor Control Node

This script provides ROS integration for controlling the motors of a Crobot robot using GPIO pins
and PWM for speed control. It subscribes to the 'keyboard_input' topic to receive commands for motor control,
and it also checks for motor load alarm triggers and emergency switch signals.

Author:

Date: 01.08.2023

ROS Node Name: crobot_motor_node

Subscribed Topics:
  - keyboard_input (std_msgs/String): Topic to receive keyboard input commands for motor control.

Published Topics:
  None

Parameters:
  speed_left_motor_pin (int): GPIO pin number for the left motor speed control.
  speed_right_motor_pin (int): GPIO pin number for the right motor speed control.
  direction_left_motor_pin (int): GPIO pin number for the left motor direction control.
  direction_right_motor_pin (int): GPIO pin number for the right motor direction control.
  stop_motor_pin (int): GPIO pin number to start the motor.
  break_motor_pin (int): GPIO pin number to break the motor.
  alarm_reset_pin (int): GPIO pin number to reset the motor load alarm.
  emergency_switch_pin (int): GPIO pin number for the emergency switch.
  alarm_trigger_pin (int): GPIO pin number for the motor load alarm trigger.
  pwm_frequency (int): Frequency (in Hz) for PWM control of the motors.

"""




import rospy
from std_msgs.msg import String
import Jetson.GPIO as gpio
import time
gpio.setwarnings(False)
class CrobotMotorNode:
    """Class for controlling motors of a Crobot robot via ROS messages.

    This class initializes the GPIO pins, sets up PWM for motor speed control,
    and subscribes to the 'keyboard_input' topic to receive commands for motor control.

    """

    def __init__(self):
        """Initialize the CrobotMotorNode class."""
        gpio.setmode(gpio.BCM)

        self.speed_left_motor_pin = 12
        self.speed_right_motor_pin = 13
        self.direction_left_motor_pin = 17
        self.direction_right_motor_pin = 27

        self.stop_motor_pin = 16#36
        self.break_motor_pin = 26#37
        self.alarm_reset_pin = 25

        self.emergency_switch_pin = 5
        self.alarm_trigger_pin = 22
        self.pwm_frequency = 1000

        self.linear_speed = 0
        self.angular_speed = 0

        # Set up GPIO pins
        gpio.setup(self.emergency_switch_pin, gpio.IN)
        gpio.setup(self.alarm_trigger_pin, gpio.IN)

        gpio.setup(self.stop_motor_pin, gpio.OUT)
        gpio.setup(self.break_motor_pin, gpio.OUT)
        gpio.setup(self.alarm_reset_pin, gpio.OUT)
        gpio.setup(self.speed_left_motor_pin, gpio.OUT)
        gpio.setup(self.speed_right_motor_pin, gpio.OUT)
        gpio.setup(self.direction_left_motor_pin, gpio.OUT)
        gpio.setup(self.direction_right_motor_pin, gpio.OUT)

        #Engaging Break and Stop pins before run
        gpio.output(self.break_motor_pin, True)
        gpio.output(self.stop_motor_pin, True)

        # Set up PWM for motor speed control
        self.speed_left_motor = gpio.PWM(self.speed_left_motor_pin, self.pwm_frequency)
        self.speed_right_motor = gpio.PWM(self.speed_right_motor_pin, self.pwm_frequency)
        self.speed_left_motor.start(0)
        self.speed_right_motor.start(0)

        # Initialize ROS node
        rospy.init_node('crobot_motor_node')

    def engage_motor(self):
        gpio.output(self.break_motor_pin, False)
        gpio.output(self.stop_motor_pin, False)

    def forward(self, speed):
        """Move the robot forward at a given speed.

            time.sleep(0.1)
        Args:
            speed (int): Speed value (0 to max) for both left and right motors.

        """
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def reverse(self, speed):
        """Move the robot backward at a given speed.

        Args:
            speed (int): Speed value (0 to max) for both left and right motors.

        """
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def left(self, speed):
        kp=0.5
        """Turn the robot left at a given angular speed.


        Args:
            speed (int): Angular speed value (0 to max) for both left and right motors.

        """
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, False)
        gpio.output(self.direction_right_motor_pin, False)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def right(self, speed):
        kp=0.5
        """Turn the robot right at a given angular speed.

        Args:
            speed (int): Angular speed value (0 to max) for both left and right motors.

        """
        self.engage_motor()
        gpio.output(self.direction_left_motor_pin, True)
        gpio.output(self.direction_right_motor_pin, True)
        self.speed_left_motor.ChangeDutyCycle(speed)
        self.speed_right_motor.ChangeDutyCycle(speed)

    def stop(self,speed):
        kp=0.5
        speed-=10
        if(speed<0):
            speed=0
        """Stop the robot by setting motor speed to 0 and turning off motor directions."""
        for i in range(speed,0,-5):
            #rospy.loginfo(i*0.5)
            self.speed_left_motor.ChangeDutyCycle(i*kp)
            self.speed_right_motor.ChangeDutyCycle(i*kp)
            time.sleep(0.2)
        gpio.output(self.stop_motor_pin, True)
        time.sleep(0.2)
        gpio.output(self.break_motor_pin, True)

    def keyboard_callback(self, msg):
        alarm_val = gpio.input(self.alarm_trigger_pin)
        emergency_val = gpio.input(self.emergency_switch_pin)
        #print(f"Alarm Value \t {alarm_val} \t emergency_val {emergency_val}")
        """Callback function for processing keyboard input commands.

        Args:
            msg (str): A string containing the command received from the 'keyboard_input' topic.

        """
        command = msg.data
        if "linear:" in command and "angular:" in command:
            # Extract the linear and angular speeds from the received message
            _, linear_speed_str, _, angular_speed_str = command.split(":")
            self.linear_speed = int(linear_speed_str)
            self.angular_speed = int(angular_speed_str)
        elif command == "up":
            rospy.loginfo("Forward")
            self.forward(self.linear_speed)  # Set the desired speed here (0-100)
        elif command == "down":
            rospy.loginfo("Reverse")
            self.reverse(self.linear_speed)
        elif command == "left":
            rospy.loginfo("Left")
            self.left(self.angular_speed)
        elif command == "right":
            rospy.loginfo("Right")
            self.right(self.angular_speed)
            rospy.loginfo(self.angular_speed)
        elif command == "stop":
            rospy.loginfo("Stop")
            if(self.angular_speed>self.linear_speed):
                self.stop(self.angular_speed)
            else:
                self.stop(self.linear_speed)
        print(f"Linear Speed {self.linear_speed}\tAngular Speed{self.angular_speed}")
    def keyboard_control(self):
        """Control the motors based on keyboard inputs.

        This method subscribes to the 'keyboard_input' topic and calls the keyboard_callback function
        when a message is received. Additionally, it checks for motor load alarm and emergency switch triggers.

        """
        rospy.Subscriber('keyboard_input', String, self.keyboard_callback)
        rospy.spin()

    def run(self):
        """Run the CrobotMotorNode class.

        This method initializes the GPIO pins, sets up PWM for motor speed control,
        and calls the keyboard_control method to start listening for keyboard inputs.

        """
        try:
            gpio.output(self.alarm_reset_pin, False)
            time.sleep(2)
            gpio.output(self.alarm_reset_pin, True)
            time.sleep(2)
            gpio.output(self.alarm_reset_pin, False)
            self.keyboard_control()
        except rospy.ROSInterruptException:
            pass
        
        self.stop(self.linear_speed)


#def Input_data():#Need To do Threading
        # if not alarm_val:
        #    rospy.loginfo("Stop")
        #    if(angular_speed>linear_speed):
        #        self.stop(self.angular_speed)
        #    else:
        #        self.stop(self.linear_speed)
        #     raise Exception("Check the Motor Load Alarm is Triggered. This code will stop here. Please restart it here.")
        # if not emergency_val:
        #    rospy.loginfo("Stop")
        #     if(angular_speed>linear_speed):
        #         self.stop(self.angular_speed)
        #     else:
        #         self.stop(self.linear_speed)
        #     raise Exception("Emergency switch is triggered. This code will stop here. Please restart it here.")
if __name__ == "__main__":
    crobot_node = CrobotMotorNode()
    crobot_node.run()
