import sys
import time
import RPi.GPIO as GPIO
import busio
import board
import adafruit_amg88xx
import numpy as np 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


#pins on the RPi which corresponds to the solenoid, servo and motor
solenoid_pin = 13
servo_pin = 16
motor_pin = 12

#controls the temperature threshold for the AMG
temperature = 30


GPIO.setup(solenoid_pin, GPIO.OUT)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(motor_pin, GPIO.OUT)

servo = GPIO.PWM(servo_pin, 50)

GPIO.output(motor_pin, GPIO.LOW)
GPIO.output(solenoid_pin, GPIO.LOW)
servo.start (0)
#FS90R adjust angle, DC motor on, rev for 5s, solenoid fire

class Launcher(Node):
        def __init__(self):
            super().__init__('Launcher')
            self.publisher_ = self.create_publisher(String,"launcher_dir",10)
        def launch(self):
            msg = String()
            counter = 0
            finishFire = 0
            rowLeft = -1
            rowRight = -1
            colTop = -1
            colBot = -1
            toFire = 0
            meanRow = -1
            meanCol = -1
            foundTop = 0
            foundBot = 0
            found = 0
            start = 0
            maxRowValues = [0 for i in range(8)]
            def upPitch():
                servo.ChangeDutyCycle(7.5)
                time.sleep(0.1)
                servo.ChangeDutyCycle(0)
            def downPitch():
                servo.ChangeDutyCycle(5)
                time.sleep(0.05)
                servo.ChangeDutyCycle(0)
            def fire():
                GPIO.output(motor_pin, GPIO.HIGH)
                time.sleep(1)
                triggerSolenoid()
                GPIO.output(motor_pin, GPIO.LOW)
                time.sleep(2)
            def triggerSolenoid():
                GPIO.output(solenoid_pin, GPIO.HIGH)
                time.sleep(0.2)
                GPIO.output(solenoid_pin, GPIO.LOW)
            try:
                while finishFire == 0:
                    transposeAmg = np.rot90(amg.pixels)
                    for i in range(8):
                        print(transposeAmg[i])
                    for i in range(8):
                        for j in range(8):
                            if(transposeAmg[i][j] >= temperature):
                                foundTop = 1
                                rowLeft = i
                                colTop= j
                                break
                        if(foundTop == 1):
                            break 
                    for i in range(7, -1, -1):
                        for j in range(7, -1, -1):
                            if(transposeAmg[i][j] >= temperature):
                                rowRight = i
                                colBot = j
                                foundBot = 1
                                break 
                        if(foundBot == 1):
                            break
                    if(foundTop == 1 and foundBot == 1):
                        found = 1;
                        print(rowLeft, colTop, rowRight, colBot)
                        if(start == 0):
                            msg.data = "start"
                            self.publisher_.publish(msg)
                            self.get_logger().info(msg.data)
                            start = 1
                        meanRow = (rowLeft + rowRight)/2
                        meanCol = (colTop + colBot)/2
                        #meanRow controls the up down pitch
                        #meanCol controls the left and right
                        if(meanRow <= 5 and meanRow >= 3 and meanCol <= 4 and meanCol >= 3):
                            toFire = 1
                            print("fire")
                        elif(meanRow >= 0 and meanCol >= 0):
                            if(meanRow < 3):
                                print("up")
                                upPitch()
                            if(meanRow > 5):
                                print("down")
                                downPitch()
                            if(meanCol < 4):
                                msg.data = "left"
                                self.publisher_.publish(msg)
                                self.get_logger().info(msg.data)
                            elif(meanCol > 4):
                                msg.data = "right"
                                self.publisher_.publish(msg)
                                self.get_logger().info(msg.data)
                    time.sleep(0.5)
                    rowLeft = -1
                    rowRight = -1
                    colTop = -1
                    rowLeft = -1
                    foundTop = 0
                    foundBot = 0
                        
                    if(toFire == 1):
                        msg.data = "fire"
                        self.publisher_.publish(msg)
                        self.get_logger().info(msg.data)
                        fire()
                        fire()
                        fire()
                        toFire = 0
                        finishFire = 1
                        
                    if(finishFire):
                        msg.data = "completed"
                        self.publisher_.publish(msg)
                        self.get_logger().info(msg.data)
            except KeyboardInterrupt:
                GPIO.cleanup()

def main(args=None):
        rclpy.init(args=args)
        launcher = Launcher()
        launcher.launch()
        launcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
