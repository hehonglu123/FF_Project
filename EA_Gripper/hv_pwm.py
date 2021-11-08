import lgpio
import time

PWMpin=12
FREQ=1000
h = lgpio.gpiochip_open(0)

try:
	while True:
		for i in range(100):
			print("%i/8 power"%i)
			value=12*i
			lgpio.tx_pwm(h,PWMpin,FREQ,i)
			time.sleep(0.7)

except KeyboardInterrupt:
	lgpio.tx_pwm(h,PWMpin,FREQ,0)
	lgpio.gpiochip_close(h)