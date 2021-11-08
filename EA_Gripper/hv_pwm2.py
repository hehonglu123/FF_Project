import RPi.GPIO as GPIO
import time

PWMpin=12
FREQ=1000

GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(PWMpin,GPIO.OUT)
pi_pwm = GPIO.PWM(PWMpin,FREQ)		#create PWM instance with frequency
pi_pwm.start(0)
pi_pwm.ChangeDutyCycle(0)
GPIO.cleanup()
# try:
# 	while True:
# 		for i in range(100):
# 			print("%i/100 power"%i)
# 			pi_pwm.ChangeDutyCycle(i)				#start PWM of required Duty Cycle 
# 			time.sleep(0.7)

# except KeyboardInterrupt:
# 	pi_pwm.ChangeDutyCycle(0)