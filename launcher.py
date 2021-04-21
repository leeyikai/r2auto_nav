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



solenoid_pin = 13
servo_pin = 16
motor_pin = 12
temperature = 25


GPIO.setup(solenoid_pin, GPIO.OUT)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(motor_pin, GPIO.OUT)

servo = GPIO.PWM(servo_pin, 50)

GPIO.output(motor_pin, GPIO.LOW)
GPIO.output(solenoid_pin, GPIO.LOW)
servo.start (0)
#FS90R adjust angle, DC motor on, rev for 5s, solenoid fire

print("wat")
class Launcher(Node):
        def __init__(self):
            super().__init__('Launcher')
            self.publisher = self.create_publisher(String,"launcher_dir",10)

   
        timer_period = 0.1
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            self.get_logger().info(msg.data)
        def launch(self):
            counter = 0
            rowLeft = -1
            rowRight = -1
            colTop = -1
            colBot = -1
            toFire = 0
            meanRow = -1
            meanCol = -1
            finishFire = 0
            foundTop = 0
            foundBot = 0
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
                    print("2")
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
                        self.publisher.publish("start")
                        self.get_logger().info("start")
                        print(rowLeft, colTop, rowRight, colBot)
                        meanRow = (rowLeft + rowRight)/2
                        meanCol = (colTop + colBot)/2
                        print(meanRow)
                        print(meanCol)
                        if(meanRow <= 4 and meanRow >= 3 and meanCol <= 3 and meanCol >= 2):
                            toFire = 1
                            print("fire")
                        elif(meanRow >= 0 and meanCol >= 0):
                            if(meanCol < 2):
                               print("up")
                            if(meanCol > 3):
                                print("down")
                            if(meanRow < 3):
                                self.publisher.publish("right")
                                self.get_logger().info("right")
                                print("right")
                            if(meanRow > 4):
                                self.publisher.publish("left")
                                self.get_logger().info("left")

                                print("left")
                    time.sleep(0.5)
                    rowLeft = -1
                    rowRight = -1
                    colTop = -1
                    rowLeft = -1
                    foundTop = 0
                    foundBot = 0

                    if(toFire == 1):
                        print("fire")
                        toFire = 0;
                        #fire()
                        finishFire = 1;

                    if(finishFire):
                        self.publisher.publish("completed")

            except KeyboardInterrupt:
                GPIO.cleanup()

def main(args=None):
        rclpy.init(args=args)
        print("start")
              launcher = Launcher()
        print("go")
        rclpy.spin(launcher)
        launcher.launch()
        launcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




                            

