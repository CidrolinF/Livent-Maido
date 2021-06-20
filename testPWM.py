import RPi.GPIO as GPIO
import time


GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

rapport = 100
p = GPIO.PWM(12, 40) #Channel, Frequence
p.start(rapport)

time.sleep(100)