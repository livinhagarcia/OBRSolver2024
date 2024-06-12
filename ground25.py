from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
display = hub.display
yaw = hub.imu

def executionDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(4,1)
    display.pixel(4,2)
    display.pixel(4,3)
def calibrateLeftDisplay(brightness):
    display.pixel(0,0,brightness)
    display.pixel(0,1,brightness)
    display.pixel(1,0,brightness)
    display.pixel(1,1,brightness)
    display.pixel(2,0,brightness)
    display.pixel(2,1,brightness)
    display.pixel(3,0,brightness)
    display.pixel(3,1,brightness)
    display.pixel(4,0,brightness)
    display.pixel(4,1,brightness)
def calibrateRightDisplay(brightness):
    display.pixel(0,3,brightness)
    display.pixel(0,4,brightness)
    display.pixel(1,3,brightness)
    display.pixel(1,4,brightness)
    display.pixel(2,3,brightness)
    display.pixel(2,4,brightness)
    display.pixel(3,3,brightness)
    display.pixel(3,4,brightness)
    display.pixel(4,3,brightness)
    display.pixel(4,4,brightness)
def proportionalAlignDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(4,1)
def axisCorrectionDisplay():
    display.off()
    display.pixel(0,2)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,3)
def intersectionSolverDisplay():
    display.off()
    display.pixel(0,2)
    display.pixel(2,2)
    display.pixel(3,2)
    display.pixel(4,2)
def recoveryTaskDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(0,3)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,2)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,3)
def desviarObsDisplay():
    display.off()
    display.pixel(0,1)
    display.pixel(0,2)
    display.pixel(1,1)
    display.pixel(1,3)
    display.pixel(2,1)
    display.pixel(2,3)
    display.pixel(3,1)
    display.pixel(3,3)
    display.pixel(4,1)
    display.pixel(4,2)

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

def axis_correction(set_point_c = 40, set_point_s = 75 ):
    axisCorrectionDisplay()
    global corner
    global name
    global move_side
    global log
    if name != "axis correction **Corner**":
        corner = 0
    if corner == 3:
        motors.stop_tank()
        if sd.reflection() < set_point_s:
            while sd.reflection() < set_point_s:
                u_value = u2.distance()
                motors.start_tank(-150,0)
                move_side = 'right'
        if se.reflection() < set_point_s : 
            while se.reflection() < set_point_s:
                u_value = u2.distance()
                motors.start_tank(0,-150)
                move_side = 'left'
        corner = 0
        name = "axis correction **Suave**"
        log = 'succeded'
    else:
        if sd.reflection() > set_point_c:
            while sd.reflection() > set_point_c:
                u_value = u2.distance()
                motors.start_tank(300,-50)
                move_side = 'left'
            corner += 1
        else:
            while se.reflection() > set_point_c:
                u_value = u2.distance()
                motors.start_tank(-50,300)
                move_side = 'right'
            corner += 1
        name = "axis correction **Corner**"
        log = 'succeded'
    updateLog()
    return [name, move_side, log]
    
       


def proportionalAlign(errorE,errorD, kP):
    proportionalAlignDisplay()
    global name
    global move_side
    global log
    name ='proportional align'
    move_side = ''
    log='failed'
    leftMotorSpd = 50 + errorE * kP * 4.7
    rightMotorSpd = 50 + errorD * kP * 4.7
    motors.start_tank(leftMotorSpd,rightMotorSpd)
    diff_l_r = leftMotorSpd - rightMotorSpd
    if diff_l_r > 0:
        move_side = 'right'
    else:
        move_side = 'left'
    log = 'succeded'
    updateLog()
    return [name, move_side, log]

def intersectionSolver(valores):
    last_values = valores
    intersectionSolverDisplay()
    print(valores)
    while valores[0] != False or valores[1] != False:
        motors.move_tank(350, 200, 200)
        last_values = valores
        valores = checkGreen(green_values)
    valores = last_values
    if valores[0] == True and valores[1] == True:
        print('dar voltinha')
        motors.move_tank(1000, 350, 350)
        while se.reflection() > 80 and sd.reflection() > 80 :
            motors.start_tank(350, -350)
        motors.stop_tank()
        wait(1000)
    else:
        if valores[0] == True:
            print('esquerdinha')
            motors.stop_tank()
            wait(1000)
            motors.start_tank(300,0)
            wait(1000)
            motors.stop_tank()
            move_side = 'left'
        else:
            print('direitinha')
            motors.stop_tank()
            wait(1000)
            while se.reflection() > 80 and sd.reflection() > 80 :
            motors.start_tank(0,300)
        motors.stop_tank()
        wait(1000)
            move_side = 'right'
    name = intersectionSolver
    log = 'succeded'
    updateLog()

def recoveryTask():
    recoveryTaskDisplay()
    global logs
    global name 
    global move_side
    global log 
    if logs[0] == "proportionalAlign":
        timeout = 0
        while se.reflection() > 20 and sc.reflection() > 20 and sd.reflection() > 20 and timeout < 3000 :
            motors.start_tank(-200,-200)
            wait(10)
            timeout += 10
        motors.stop_tank()
        if se.reflection() < 20 or sc.reflection() < 20 or sd.reflection() < 20:
            name = "recovery task"
            move_side = move_side
            log = "succeded"
            updateLog()
            return "succeded"
        else:
            name = "recovery task"
            move_side = move_side
            log = "failed"
            updateLog()
            return "succeded"
    if logs[0] == "axis correction" or logs[0] == "intersection solver":
        if logs[0] == "intersection solver":
            if logs[1] == "right":
                logs[1] = "left"
            if logs[1] == "left":
                logs[1] = "right"
        yaw.reset_heading()
        if logs[1] == "right":
            motors.start_tank(-200,200)
            while yaw.heading() >= -90:
                if se.reflection() < 20 or sd.reflection() < 20:
                    name = "recovery task"
                    move_side = "left"
                    log = "succeded"
                    updateLog()
                    return "succeded"
                else:
                    timeout = 0
                    while se.reflection() > 20 and sc.reflection() > 20 and sd.reflection() > 20 and timeout < 3000 :
                        motors.start_tank(-200,-200)
                        wait(10)
                        timeout += 10
                    motors.stop_tank()
                    if se.reflection() < 20 or sc.reflection() < 20 or sd.reflection() < 20:
                        name = "recovery task"
                        move_side = move_side
                        log = "succeded"
                        updateLog()
                        return "succeded"
                    else:
                        name = "recovery task"
                        move_side = move_side
                        log = "failed"
                        updateLog()
                        return "succeded"
            yaw.reset_heading()
            motors.start_tank(200,-200)
            while yaw.heading() <= 90:
                if se.reflection() < 20 or sd.reflection() < 20:
                    name = "recovery task"
                    move_side = "right"
                    log = "succeded"
                    updateLog()
                    return "succeded"
                else:
                    timeout = 0
                    while se.reflection() > 20 and sc.reflection() > 20 and sd.reflection() > 20 and timeout < 3000 :
                        motors.start_tank(-200,-200)
                        wait(10)
                        timeout += 10
                    motors.stop_tank()
                    if se.reflection() < 20 or sc.reflection() < 20 or sd.reflection() < 20:
                        name = "recovery task"
                        move_side = move_side
                        log = "succeded"
                        updateLog()
                        return "succeded"
                    else:
                        name = "recovery task"
                        move_side = move_side
                        log = "failed"
                        updateLog()
                        return "succeded"
        if logs[1] == "left":
            motors.start_tank(200,-200)
            while yaw.heading() <= 90:
                if se.reflection() < 20 or sd.reflection() < 20:
                    name = "recovery task"
                    move_side = "right"
                    log = "succeded"
                    updateLog()
                    return "succeded"
                else:
                    timeout = 0
                    while se.reflection() > 20 and sc.reflection() > 20 and sd.reflection() > 20 and timeout < 3000 :
                        motors.start_tank(-200,-200)
                        wait(10)
                        timeout += 10
                    motors.stop_tank()
                    if se.reflection() < 20 or sc.reflection() < 20 or sd.reflection() < 20:
                        name = "recovery task"
                        move_side = move_side
                        log = "succeded"
                        updateLog()
                        return "succeded"
                    else:
                        name = "recovery task"
                        move_side = move_side
                        log = "failed"
                        updateLog()
                        return "succeded"
            yaw.reset_heading()
            motors.start_tank(-200,200)
            while yaw.heading() >= -90:
                if se.reflection() < 20 or sd.reflection() < 20:
                    name = "recovery task"
                    move_side = "left"
                    log = "succeded"
                    updateLog()
                    return "succeded"
                else:
                    timeout = 0
                    while se.reflection() > 20 and sc.reflection() > 20 and sd.reflection() > 20 and timeout < 3000 :
                        motors.start_tank(-200,-200)
                        wait(10)
                        timeout += 10
                    motors.stop_tank()
                    if se.reflection() < 20 or sc.reflection() < 20 or sd.reflection() < 20:
                        name = "recovery task"
                        move_side = move_side
                        log = "succeded"
                        updateLog()
                        return "succeded"
                    else:
                        name = "recovery task"
                        move_side = move_side
                        log = "failed"
                        updateLog()
                        return "succeded"
                
def getGreenValues(sensor,side):
    hsv_min = [0,0,0]
    hsv_max = [0,0,0]
    for x in range(200):
        if side == "direita":
            calibrateRightDisplay(int((x+1)/2))
        if side == "esquerda":
            calibrateLeftDisplay(int((x+1)/2))
        hsv_obj = sensor.hsv()
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
    sensor_d = sd.hsv()
    sensor_e = se.hsv()
    direita = False
    esquerda = False
    # print(sensor)
    # if sensor.h < values[0][0] or sensor.h > values[1][0]:
    #     return False
    # if sensor.s < values[0][1] or sensor.s > values[1][1]:
    #     return False
    # if sensor.v < values[0][2] or sensor.v > values[1][2]:
    #     return False
    if sensor_d.h > values[0][0] and sensor_d.h < values[1][0]:
        if sensor_d.s > values[0][1] and sensor_d.s < values[1][1]:
            if sensor_d.v > values[0][2] and sensor_d.v < values[1][2]:
                direita = True
    if sensor_e.h > values[0][0] and sensor_e.h < values[1][0]:
        if sensor_e.s > values[0][1] and sensor_e.s < values[1][1]:
            if sensor_e.v > values[0][2] and sensor_e.v < values[1][2]:
                esquerda = True
    return [esquerda, direita]

def desviarObs(lado = 'right'):
    desviarObsDisplay()
    if lado == 'right':
        if name == 'axis correction **Corner**':
            print("DESVIO axis")
            motors.move_tank(800, -400, 400)
            motors.stop_tank()  
            motors.move_tank(1400, 500, 200)
            motors.move_tank(700, 200, 200)
            motors.move_tank(1400, 500, 130)
            motors.move_tank(300, 100, 100)
        elif name == 'proportional align':
            print("DESVIO proportional")
        motors.move_tank(800, -400, 400)
        motors.stop_tank()  
        motors.move_tank(1850, 500, 200)
        motors.move_tank(700, 200, 200)
        motors.move_tank(1850, 500, 180)
        while se.reflection() > 80 and sd.reflection() > 80 :
            motors.start_tank(350, 150)
        motors.stop_tank()
        wait(1000)
        return True
    elif lado == 'left':
        motors.move_tank(1300, 400, -400)
        motors.stop_tank()
        while se.reflection() > 80 or sd.reflection() > 80 :
            motors.start_tank(150, 350)
        motors.stop_tank()
        wait(1000)
        return True
    return False       

def checarResgate(u_value):
    global logs
    global name 
    global move_side
    global log 
    if u_value > 900 and u_value < 930:
        motors.stop_tank()
        hub.speaker.beep()
        motors.move_tank(3000,250,250)
        motors.move_tank(1500,-250,250)
        motors.move_tank(1000,250,250)
        wait(2000)
        u_value = u0
        u2.distance()
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
        motors.stop_tank
        move_side = 'right'
        if desviarObs(move_side) == True:
            name = "desvio Obs"
            log = 'succeded'
            updateLog()
        return True
    
    return False


hub = PrimeHub()

setPoint = 50
darkest = ""

# defining motors
motors = MotorPair(Port.A,Port.B)

# defining sensors
u2 = UltrasonicSensor(Port.E)
sc = ColorSensor(Port.D)
sd = ColorSensor(Port.C)
se = ColorSensor(Port.F)

# defining the log list
name = 'Beginning run'
move_side = 'None'
log = 'succeded'
logs = [name,move_side,log]
corner = 0
green_values = [[162.5, 24.5, 43.5], [177.5, 87.5, 100.0]]
mode = ""

if __name__ == "__main__":
    while True:
        if hub.buttons.pressed() == {Button.LEFT}:
            mode = "execution"
        if hub.buttons.pressed() == {Button.RIGHT}:
            mode = "calibrate"
        if mode == "calibrate":
            print("------calibrando------")
            leftValues = getGreenValues(se,"esquerda")
            rightValues = getGreenValues(sd,"direita")
            minMedian = [(leftValues[0][0] + rightValues[0][0]) / 2, (leftValues[0][1] + rightValues[0][1]) / 2, (leftValues[0][2] + rightValues[0][2]) / 2]
            maxMedian = [(leftValues[1][0] + rightValues[1][0]) / 2, (leftValues[1][1] + rightValues[1][1]) / 2, (leftValues[1][2] + rightValues[1][2]) / 2]
            green_values = [minMedian,maxMedian]
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
                if se_value > 50 and sd_value > 50 and sc_value < 45:
                    print(sc_value)
                    proportionalAlign(errore,errord,1.2)
                else:
                    valores_verdes = checkGreen(green_values)
                    if valores_verdes[0] != False or valores_verdes[1] != False:
                        print('-------- Green founded --------')
                        intersectionSolver(valores_verdes)
                    if se_value > 80 and sd_value > 80 and sc_value > 80:
                        if logs[0] == 'proportional align':
                            print('------- Possible Gap ---------')
                        else:
                            recoveryTask()
                    else:
                        motors.stop_tank()
                        if se_value > sd_value: 
                            darkest = "direita"
                        if sd_value > se_value:
                            darkest = "esquerda"
                        if se_value < 45 and sd_value < 45:
                            motors.move_tank(300, 200, 200)
                            print("------ crossing line ------")
                            valores_verdes = checkGreen(green_values)
                            if valores_verdes[0] != False or valores_verdes[1] != False:
                                print('-------- Green founded --------')
                                intersectionSolver(valores_verdes)
                            motors.move_tank(2000, 200, 200)
                            se_value = se.reflection()
                            sd_value = sd.reflection()
                            sc_value = sc.reflection()
                            errord = se_value - setPoint
                            errore = sd_value - setPoint
                            if se_value > 60 and sd_value > 60 and sc_value < 45:
                                proportionalAlign(errore,errord,0.8)
                            else:
                                print('back until see black')
                                motors.move_tank(2000, -200, -200)
                                print('Axis Correction')
                                axis_correction()
                        else:
                            if darkest == "esquerda":        
                                print('Axis Correction')
                                axis_correction()
                            if darkest == "direita":        
                                print('Axis Correction')
                                axis_correction()

                
                msg = str(se.reflection()) + ',' + str(sc.reflection()) + ',' + str(sd.reflection())
                print(msg)
