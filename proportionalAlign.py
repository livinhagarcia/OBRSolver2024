from spike_pybricks_motorpair import MotorPair
from pybricks.parameters import Port
from pybricks.hubs import PrimeHub

hub = PrimeHub()
display = hub.display

motors = MotorPair(Port.A,Port.B)

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

def proportionalAlign(errorE,errorD, kP):
    proportionalAlignDisplay()
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
    return [name, move_side, log]