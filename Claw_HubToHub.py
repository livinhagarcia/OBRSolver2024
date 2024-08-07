from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub(broadcast_channel=2, observe_channels=[1])

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
                self.motor2.run((speed2))
        self.motor1.stop()
        self.motor2.stop()
        return "succeded"
    def move_tank(self,amount, speed1, speed2):    
        self.motor1.run(speed1)
        self.motor2.run((speed2))
        wait(amount)
        self.motor1.stop()
        self.motor2.stop()
    def start_tank(self, speed1, speed2):
        self.motor1.run(speed1)
        self.motor2.run((speed2))           
    def stop_tank(self):
        self.motor1.run(0)
        self.motor2.run(0)

class Claw:
    def __init__(self,P1,P2,Hand):
        self.clawPair = MotorPair(P1,P2)
        self.hand = Motor(Hand)
    def up(self):
        self.clawPair.move_tank(1000,-200,200)
        print("up")
    def down(self):
        self.clawPair.move_tank(700,500,-500)
        print("down")
    def open(self):
        self.hand.run_angle(-500,20)
    def close(self):
        self.hand.run_angle(500,30)
        print("close")
    def pickUp(self):
        self.down()
        wait(1000)
        self.close()
        self.up()
    def release(self):
        self.open()
        self.down()
        wait(1000)
        self.up()

claw = Claw(Port.A, Port.C, Port.D)
exec = False
while True:
    hub.display.pixel(1,1)
    hub.light.on(Color.RED)

    if hub.buttons.pressed() == {Button.LEFT}:
        hub.ble.broadcast(1)
        wait(1000)
        hub.ble.broadcast(0) #reset
        exec = True
    if hub.buttons.pressed() == {Button.RIGHT}:
        hub.ble.broadcast(2)
        wait(1000)
        hub.ble.broadcast(0) #reset
    if exec :    
        if hub.buttons.pressed() == {Button.LEFT} or hub.buttons.pressed() == {Button.RIGHT}:
            hub.ble.broadcast(3)
            print("hereeeeeeeee")
            wait(1000)
            hub.ble.broadcast(0) #reset

    inf = hub.ble.observe(1)
    print(inf)
    if inf == 0:
        claw.pickUp()

    elif inf == 1:
        claw.release()
    inf = None
