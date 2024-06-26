from displays import desviarObsDisplay
from spike_pybricks_motorpair import MotorPair
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.pupdevices import ColorSensor, UltrasonicSensor

motors = MotorPair(Port.A,Port.B)
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)

def desviarObs(lado = 'right'):
    name = ''
    desviarObsDisplay()
    if lado == 'right':
        if name == 'axis correction **Corner**':
            motors.move_tank(800, -400, 400)
            motors.stop_tank()  
            motors.move_tank(1400, 500, 200)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1400, 500, 130)
            motors.move_tank(300, 100, 100)
        elif name == 'proportional align':
            motors.move_tank(800, -400, 400)
            motors.stop_tank()  
            motors.move_tank(1850, 500, 200)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1850, 500, 180)
        while se.reflection() > 80 and sd.reflection() > 80 :
            motors.start_tank(350, 150)
        motors.stop_tank()
        wait(1000)
        return [name, lado, 'succeded']
    elif lado == 'left':
        motors.move_tank(1300, 400, -400)
        motors.stop_tank()
        while se.reflection() > 80 or sd.reflection() > 80 :
            motors.start_tank(150, 350)
        motors.stop_tank()
        wait(1000)
        return [name, lado, 'succeded']
    return [name, lado, 'failed']    