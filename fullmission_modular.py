from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from spike_pybricks_motorpair import MotorPair
from proportionalAlign import proportionalAlign
from axis_correction import axis_correction
from desviar_obs import desviarObs
from intersection import Intersection
from displays import *

hub = PrimeHub()
display = hub.display
yaw = hub.imu



def updateLog(log):
    global logs
    if len(log) != 3:
        return False
    if log != logs[-1]:
        logs.append([name,move_side,log])
        return True
    
def recoveryTask():
    return 1


def checarResgate(u_value):
    if u_value > 900 and u_value < 930:
        motors.stop_tank()
        hub.speaker.beep()
        motors.move_tank(3000,250,250)
        motors.move_tank(1500,-250,250)
        motors.move_tank(1000,250,250)
        wait(2000)
        u_value = u2.distance()
        print(u_value)
        #Mover motor esquerdo para frente fazendo com q o robo gire 90 graus
        if u_value > 500 and u_value < 800:
            hub.speaker.beep()
            hub.display.pixel(4,4)
            print("resgate!!!!")
            return True
        else:
            motors.move_tank(1000,-250,-250)
            motors.move_tank(1500,250,-250)
            motors.move_tank(2250,-250,-250)
    elif u_value < 100:
        hub.speaker.beep()
        motors.stop_tank()
        move_side = 'right'
        desviarObs()
    return False


hub = PrimeHub()

setPoint = 50
darkest = ""

# defining motors
motors = MotorPair(Port.A,Port.B)

# defining sensors
green_values = [[[144.18, 59.945, 52.26], [184.18, 99.945, 92.26]], [[146.77, 56.265, 53.105], [186.77, 96.265, 93.105]]]
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)
i = Intersection(se, sd, green_values)
# defining the log list
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
logs = [[name,move_side,log]]
corner = 0

mode = ""

if __name__ == "__main__":
    while True:
        if hub.buttons.pressed() == {Button.LEFT}:
            mode = "execution"
        if hub.buttons.pressed() == {Button.RIGHT}:
            mode = "calibrate"
        if mode == "calibrate":
            print("------calibrando------")
            leftValues = i.getGreenValues("esquerda")
            rightValues = i.getGreenValues("direita")
            # minMedian = [(leftValues[0][0] + rightValues[0][0]) / 2, (leftValues[0][1] + rightValues[0][1]) / 2, (leftValues[0][2] + rightValues[0][2]) / 2]
            # maxMedian = [(leftValues[1][0] + rightValues[1][0]) / 2, (leftValues[1][1] + rightValues[1][1]) / 2, (leftValues[1][2] + rightValues[1][2]) / 2]
            # green_values = [minMedian,maxMedian]
            green_values = [leftValues, rightValues]
            print(green_values)
            display.off()
            mode = ""
        if mode == "execution":
            executionDisplay()
            u_value = u2.distance()
            while checarResgate(u_value) == False:
                print(logs,corner)
                u_value = u2.distance()
                se_value = se.reflection()
                sd_value = sd.reflection()
                sc_value = sc.reflection()
                errord = se_value - setPoint
                errore = sd_value - setPoint
                if se_value > 50 and sd_value > 50 and sc_value < 55:
                    updateLog(proportionalAlign(errore,errord,1.2))
                else:
                    valores_verdes = i.checkGreen(green_values)
                    if valores_verdes[0] != False or valores_verdes[1] != False:
                        updateLog(i.intersectionSolver(valores_verdes))
                    if se_value > 80 and sd_value > 80 and sc_value > 80:
                        if logs[0] == 'proportional align':
                            updateLog(["gap", None, "succeded"])
                        else:
                            recoveryTask()
                    else:
                        motors.stop_tank()
                        if se_value > sd_value: 
                            darkest = "direita"
                        if sd_value > se_value:
                            darkest = "esquerda"
                        if se_value < 30 and sd_value < 30:
                            motors.move_tank(300, 200, 200)
                            # valores_verdes = checkGreen(green_values)
                            # if valores_verdes[0] != False or valores_verdes[1] != False:
                            #     print('-------- Green founded --------')
                            #     intersectionSolver(valores_verdes)
                            motors.move_tank(2000, 200, 200)
                            se_value = se.reflection()
                            sd_value = sd.reflection()
                            sc_value = sc.reflection()
                            errord = se_value - setPoint
                            errore = sd_value - setPoint
                            if se_value > 60 and sd_value > 60 and sc_value < 45:
                                updateLog(proportionalAlign(errore,errord,0.8))
                            else:
                                print('back until see black')
                                motors.move_tank(2200, -200, -200)
                                updateLog(axis_correction())
                        else:
                            if darkest == "esquerda":  
                                updateLog(axis_correction())
                            if darkest == "direita":    
                                updateLog(axis_correction())
                msg = str(se.reflection()) + ',' + str(sc.reflection()) + ',' + str(sd.reflection())
                print(msg)