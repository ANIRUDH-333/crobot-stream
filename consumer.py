
import pika
import os
import json
import rospy
from std_msgs.msg import String
import Jetson.GPIO as gpio
import time

gpio.setwarnings(False)
class MotorControl:
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

        gpio.output(self.alarm_reset_pin, False)
        time.sleep(2)
        gpio.output(self.alarm_reset_pin, True)
        time.sleep(2)
        gpio.output(self.alarm_reset_pin, False)

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


motor_controller = MotorControl()

# Replace these variables with your RabbitMQ server details
amqps_url = 'amqps://crobot:passiscrobot@b-f19df6ec-ed5f-4427-bff3-b81ee7fbc645.mq.ap-south-1.amazonaws.com:5671'
queue_name = 'keystoke'

def callback(ch, method, properties, body):
    message = body.decode()
    print("Received message:", message)

    # Implement your logic based on the message content
    if message == 'ArrowUp':
        motor_controller.forward(motor_controller.linear_speed)
    elif message == 'ArrowDown':
        motor_controller.reverse(motor_controller.linear_speed)
    elif message == 'ArrowRight':
        motor_controller.right(motor_controller.angular_speed)
    elif message == 'ArrowLeft':
        motor_controller.left(motor_controller.angular_speed)
    elif message == 's':
        if(motor_controller.angular_speed>motor_controller.linear_speed):
            motor_controller.stop(motor_controller.angular_speed)
        else:
            motor_controller.stop(motor_controller.linear_speed)
    # Add more conditions as needed

def main():
    # Set up a connection and channel
    params = pika.URLParameters(amqps_url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    # Declare the queue
    channel.queue_declare(queue=queue_name)

    # Set up subscription on the queue
    channel.basic_consume(queue=queue_name, on_message_callback=callback, auto_ack=True)

    print('Waiting for messages. To exit press CTRL+C')
    channel.start_consuming()

if __name__ == '__main__':
    main()

