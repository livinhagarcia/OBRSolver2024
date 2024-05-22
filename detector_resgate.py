from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel=1)

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

# right_motor = Motor(Port.A)
# left_motor = Motor(Port.B)
motors = MotorPair(Port.A,Port.B)
u2 = UltrasonicSensor(Port.E)

while True:
    u_value = u2.distance()
    print(u_value)
    while u_value < 820 or u_value > 840:
        u_value = u2.distance()
        print(u_value)
        motors.start_tank(200,200)
    motors.stop_tank()
    hub.speaker.beep()
    motors.move_tank(2500,250,250)
    motors.move_tank(1500,-250,250)
    u_value = u2.distance()
    print(u_value)
    #Mover motor esquerdo para frente fazendo com q o robo gire 90 graus
    if u_value > 500 and u_value < 650:
        hub.speaker.beep()
        # print(hub.buttons.pressed())
        # while not len(hub.buttons.pressed()) > 0 :
        #     print(u2.distance())
        # wait(2000)
        # u_value = u2.distance()
        # #mover motor direito e esquero fazendo com q o robô gire 180 graus
        # if u_value > 30 and u_value < 60:
        #     hub.speaker.beep()
        #     while not len(hub.buttons.pressed()) > 0 :
        #         print(u2.distance())
        print("resgate!!!!")
    else:
        motors.move_tank(1500,250,-250)
        motors.move_tank(2250,-250,-250)
    break
#detecção do resgate
