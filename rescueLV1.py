from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath as math

u2 = UltrasonicSensor(Port.E)
u_value = u2.distance()

hub = PrimeHub()

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath as math


hub = PrimeHub()

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
        self.motor1.hold()
        self.motor2.hold()

class myMap:
    def __init__ (self,start_pos):
        self.start_pos = start_pos
        pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
        self.points = [pos]

    def addPoint(self,point):
        pos = [point[0], point[1], point[2]]
        if not pos in self.points:
            self.points += [pos]

class Robot:
    def __init__ (self,motorE,motorD,force_sensor_port = None,position = [0,0,0]):
        self.position = position
        self.map = myMap(self.position)
        self.motors = MotorPair(motorE, motorD)
        self.hub = PrimeHub()
        self.hub.imu.reset_heading(0)
        if force_sensor_port:
            self.force_sensor = ForceSensor(force_sensor_port)

    def pointTo(self, degrees, precision = 3):
        dir = self.position[2]
        # print(range(degrees - precision, degrees + precision))
        if(dir < degrees):
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                print(self.hub.imu.heading())
                self.motors.start_tank(-200,200)
            self.motors.stop_tank()
        else:
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                self.motors.start_tank(200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = self.hub.imu.heading()
        self.map.points[-1][2] = self.position[2]
    
    def back_pointTo(self, degrees, precision = 3):
        dir = self.position[2]
        if degrees < 0:
            degrees += 180
        else:
            degrees -= 180
        # degrees = degrees * -1 
        # print(range(degrees - precision, degrees + precision))
        if(dir < degrees):
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                print(self.hub.imu.heading())
                self.motors.start_tank(-200,200)
            self.motors.stop_tank()
        else:
            while self.hub.imu.heading() < (degrees - precision) or self.hub.imu.heading() > (degrees + precision):
                self.motors.start_tank(200,-200)
            self.motors.stop_tank()
        print('turning ' + str(degrees) + ' degrees')
        self.position[2] = self.hub.imu.heading()
        self.map.points[-1][2] = self.position[2]


    def calibrateDirTo(self, goal, precision = 1):
        dir = self.position[2]
        if dir < goal:
            while not self.hub.imu.heading() == goal:
                real_value = self.hub.imu.heading()
                diff = goal - real_value
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        else:
            while not self.hub.imu.heading() == goal:
                real_value = self.imu.heading()
                diff = real_value - goal
                print(str(real_value) + " missing " + str(diff) + " to turn ")
        real_value = self.hub.imu.heading()
        print('dir calibrated to ' + str(real_value) + ' degrees, the goal was ' + str(goal))
        self.position[2] = self.hub.imu.heading()
        self.map.points[-1][2] = self.position[2]

    def moveX(self,q):
        if q:
            if q > 0:
                self.pointTo(90)
            else:
                self.pointTo(-90)
            self.motors.move_angle(abs(q), 200, 200)
            print('moving ' + str(q) + ' on X')
            self.position[0] += q
            self.map.addPoint(self.position)

    def moveY(self,q):
        if q :
            if q > 0:
                self.pointTo(0)
            else:
                self.pointTo(178)
            self.motors.move_angle(abs(q),200, 200)
            print('moving ' + str(q) + ' on Y')
            self.position[1] += q
            self.map.addPoint(self.position)
    
    def back_goTo(self,x,y):
        x = int( x + (-1*self.position[0]) )
        y = int( y + (-1*self.position[1]) )
        dist = math.sqrt(x**2 + y**2)
        if x != 0 and y > 0:
            self.back_pointTo(int(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.back_pointTo(90 + int(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.back_pointTo(-90 - int(abs(math.degrees(math.atan(y/x)))))
        elif x == 0 and y != 0:
            if y > 0:
                self.back_pointTo(0)
            else:
                self.back_pointTo(180)
        elif x != 0 and y == 0:
            if x > 0:
                self.back_pointTo(90)
            else:
                self.back_pointTo(-90)
        else:
            return 0
        self.motors.move_angle(dist,-200, -200)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

    def goTo(self,x,y):
        x = int( x + (-1*self.position[0]) )
        y = int( y + (-1*self.position[1]) )
        dist = math.sqrt(x**2 + y**2)
        if x != 0 and y > 0:
            self.pointTo(int(math.degrees(math.atan(x/y))))
        elif x > 0 and y < 0:
            self.pointTo(90 + int(abs(math.degrees(math.atan(y/x)))))
        elif x < 0 and y < 0:
            self.pointTo(-90 - int(abs(math.degrees(math.atan(y/x)))))
        elif x == 0 and y != 0:
            self.moveY(y)
            return 1
        elif x != 0 and y == 0:
            self.moveX(x)
            return 1
        else:
            return 0
        self.motors.move_angle(dist,200, 200)
        self.position[0] += x
        self.position[1] += y
        self.map.addPoint(self.position)
        return 1

    def doRoute(self, pointlist, goandback = False, back = False):
        for point in pointlist:
            if back:
                self.back_goTo(point[0],point[1])    
            else:
                self.goTo(point[0],point[1])
        if goandback == True:
            lista = self.map.points.copy()
            lista.reverse()
            self.doRoute(lista, False, back)

def FindSafe(areas):
    global PontoInicial
    pos_areas = areas
    print(PontoInicial)
    if [PontoInicial[0],PontoInicial[1]] in pos_areas:
        print("plmrdedeus")
        pos_areas.pop(pos_areas.index([PontoInicial[0],PontoInicial[1]]))
    else:
        print(PontoInicial)
        print([PontoInicial[0],PontoInicial[1]])
        print('N TA HEIN')
    if [out[0],out[1]] in pos_areas:
        pos_areas.pop(pos_areas.index([out[0],out[1]]))
    for area in pos_areas:
        print("indo para:" + str(area))
        robo.goTo(area[0], area[1])
        leitura = sc.hsv()
        # print(sc.hsv())
        if [leitura.h,leitura.s,leitura.v] >= [155,80,60] and [leitura.h,leitura.s,leitura.v] <= [175,100,85]:
            hub.speaker.beep()
            robo.motors.stop_tank()
            wait(2000)
            return area
    return False

safe = None
ListaPontos = [[385,385],[1155,385],[1925,385]]
#Saídas = [[385,0],[1155,0],[1925,0],[385,2310][1155,2310],[1925,2310],[0,385],[0,1155],[0,1925],[2310,385],[2310,1155],[2310,1925]]
PontoInicial = [1925,385,0]
Center = [1155,1155]
AreaResgate = [[385,385],[385,1925],[1925,1925],[1925,385]]
out = [385,385,-90]
sc = ColorSensor(Port.D)
robo = Robot(Port.A, Port.B, None, PontoInicial)
def resgate():
    print(PontoInicial)

    robo.goTo(Center[0], Center[1])
    
    safe = FindSafe(AreaResgate)
    if not safe:
        robo.goTo(Center[0], Center[1])
        robo.goTo(out[0], out[1])
        robo.pointTo(out[2])
        wait(1000)
        robo.motors.move_tank(1000,500,500)    
    else:
        robo.back_goTo(Center[0], Center[1])
        robo.goTo(out[0], out[1])
        robo.pointTo(out[2])
        wait(1000)
        robo.motors.move_tank(1000,500,500)

    print(robo.map.points)
