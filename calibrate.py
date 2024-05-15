from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

sd = ColorSensor(Port.C)

def getGreenValues():
    hsv_min = [0,0,0]
    hsv_max = [0,0,0]

    for x in range(200):
        hsv_obj = sd.hsv()
        if hsv_obj.h < hsv_min[0] or hsv_min[0] == 0 :
            hsv_min[0] = hsv_obj.h
        if hsv_obj.s < hsv_min[1] or hsv_min[1] == 0 :
            hsv_min[1] = hsv_obj.s
        if hsv_obj.v < hsv_min[2] or hsv_min[2] == 0 :
            hsv_min[2] = hsv_obj.v
        if hsv_obj.h > hsv_max[0]:
            hsv_max[0] = hsv_obj.h
        if hsv_obj.s > hsv_max[1]:
            hsv_max[1] = hsv_obj.s
        if hsv_obj.v > hsv_max[2]:
            hsv_max[2] = hsv_obj.v
        wait(50)
    hsv_values = [hsv_min, hsv_max]
    print(hsv_values)
    return hsv_values

def checkGreen(values):
    sensor = sd.hsv()
    # print(sensor)
    if sensor.h < values[0][0] or sensor.h > values[1][0]:
        return False
    if sensor.s < values[0][1] or sensor.s > values[1][1]:
        return False
    if sensor.v < values[0][2] or sensor.v > values[1][2]:
        return False
    return True

if __name__ == '__main__':
    hsv_green = getGreenValues()
    wait(2000)
    while len(hub.buttons.pressed()) == 0 :
        print(checkGreen(hsv_green))
    print(hsv_green)
