from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class MotorPair:
    def __init__(self, port1, port2):
        self.motor1 = Motor(port1)
        self.motor2 = Motor(port2)
        self.timer = StopWatch()
    def move_angle(self,amount,speed1,speed2, timeout = 1000):
        self.motor1.reset_angle(0)
        self.motor2.reset_angle(0)
        self.timer.reset()
        while abs(self.motor1.angle()) < amount or self.timer.time() < timeout:
            while abs(self.motor2.angle()) < amount or self.timer.time() < timeout:
                self.motor1.run(speed1)
                self.motor2.run(-(speed2))
        self.motor1.stop()
        self.motor2.stop()
        return "succeded"
    def move_tank(self,amount, speed1, speed2):    
        self.motor1.run(speed1)
        self.motor2.run(-(speed2))
        wait(amount)
        self.motor1.stop()
        self.motor2.stop()
    def start_tank(self, speed1, speed2):
        self.motor1.run(speed1)
        self.motor2.run(-(speed2))
    def stop_tank(self):
        self.motor1.run(0)
        self.motor2.run(0)


def updateLog():
    global logs
    global name
    global move_side
    global log 
    logs = [name,move_side,log]

def axis_correction(set_point_c = 40, set_point_s = 35 ):
    global corner
    global name
    global move_side
    global log
    if corner == 3:
        motors.stop_tank()
        corner = 0
        if sd.reflection() < set_point_s:
            while sd.reflection() < set_point_s:
                motors.start_tank(-200,50)
                move_side = 'right'
        else: 
            while sd.reflection() < set_point_s:
                motors.start_tank(50,-200)
                move_side = 'left'
        name = "axis correction **Suave**"
        log = 'succeded'
    else:
        if sd.reflection() > set_point_c:
            while sd.reflection() > set_point_c:
                motors.start_tank(300,-50)
                move_side = 'left'
            corner += 1
        else:
            while se.reflection() > set_point_c:
                motors.start_tank(-50,300)
                move_side = 'right'
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    updateLog()
    return [name, move_side, log]
    
       


def proportionalAlign(errorE,errorD, kP):
    global name
    global move_side
    global log
    name ='proportionalAlign'
    move_side = ''
    log='failed'
    leftMotorSpd = errorE * kP * 4.7
    rightMotorSpd = errorD * kP * 4.7
    motors.start_tank(leftMotorSpd,rightMotorSpd)
    diff_l_r = leftMotorSpd - rightMotorSpd
    if diff_l_r > 0:
        move_side = 'right'
    else:
        move_side = 'left'
    log = 'succeded'
    updateLog()
    return [name, move_side, log]
# def intersectionSolver():

def checkGreen():
    ve = se.color() == 'green'
    vd = se.color() == 'green'
    if ve and vd:
        return True
    else:
        if ve:
            return True
        if vd: 
            return True
    return False



hub = PrimeHub()

setPoint = 50

# defining motors
motors = MotorPair(Port.A,Port.B)

# defining sensors
u2 = UltrasonicSensor(Port.E).distance()
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)

# defining the log list
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
logs = [name,move_side,log]
corner = 0

if __name__ == "__main__":
    while 1:
        se_value = se.reflection()
        sd_value = sd.reflection()
        sc_value = sc.reflection()
        errord = se_value - setPoint
        errore = sd_value - setPoint
        if se_value > 60 and sd_value > 60:
            proportionalAlign(errore,errord,0.8)
        else:
            if checkGreen():
                print('-------- Green founded --------')
            else:
                if se_value > 80 and sd_value > 80 and sc_value > 80:
                    if logs[0] == 'proportionalAlign':
                        print('------- Possible Gap ---------')
                    else:
                        print("xxxxxxxxx  Recovery  xxxxxxxxx")
                else:
                    if se_value < 30 and sd_value < 30:
                        print("------ crossing line -----")
                        motors.move_tank(10, 200, 200)
                        se_value = se.reflection()
                        sd_value = sd.reflection()
                        sc_value = sc.reflection()
                        errord = se_value - setPoint
                        errore = sd_value - setPoint
                        if errord > 40 and errore > 40:
                            proportionalAlign(errore,errord,0.8)
                        else:
                            print('back until see black')
                            print('Axis Correction')
                            axis_correction()
                    elif se_value < 30 or sd_value < 30:
                        print('Axis Correction\n\n\n')
                        axis_correction()
        print(errore,errord)
        print(logs)
