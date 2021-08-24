from gpiozero import Button
from time import sleep

button = Button(17, pull_up=False)

#17: detection sensor
#22: mechanical switch

while True:
    if button.is_pressed:
        print("Pressed")
    else:
        print("Released")
    sleep(1)