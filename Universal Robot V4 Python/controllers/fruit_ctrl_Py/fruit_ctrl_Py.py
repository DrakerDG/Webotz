"""fruit_ctrl_Py controller."""

from controller import Supervisor
import random

random.seed()

robot = Supervisor()

timestep = 64

i = 0
j = 8
k = 8
fr = 1
max = 50

timer = 0
tmrbk = robot.getTime()

# Initialize gate motor
gate = robot.getDevice('gate')

# Initialize IR sensor of the gate
ir0 = robot.getDevice('ir0')
ir0.enable(timestep)

# Initialize LED of the gate
led = robot.getDevice('led')
led.set(1)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Initial position of fruits
fruit_initial_translation = [0.570002,2.85005,0.349962]

# Getting the field of the conveyor belt speed
cb = robot.getFromDef('cb')
cb_field = cb.getField('speed')

# Getting the field of the control panel url
cp = robot.getFromDef('CP')
cp_field = cp.getField('url')

def gateCtrl(speed):
    if ir0.getValue() > 360 or speed == 0:
        gate.setPosition(0)
        led.set(1)
    else:
        gate.setPosition(-0.349066)
        led.set(0)

# Main loop:
while robot.step(timestep) != -1:

    # Get the conveyor belt speed
    speed = cb_field.getSFFloat()

    gateCtrl(speed)
    
    if speed > 0:
        if int(robot.getTime()) != tmrbk and timer < 10:
            tmrbk = int(robot.getTime())
            timer += 1
            
        if timer > 8:
            if i == 0:
                if fr > 0:
                    fr = int(random.choice([1,2]))
                    if j == max: fr = 2
                    if k == max: fr = 1
                    if fr == 1 and j < max:
                        name = f'apple{j:d}'
                        j += 1
                    if fr == 2 and k < max:
                        name = f'orange{k:d}'
                        k += 1
                    fruit = robot.getFromDef(name)
                    fruit_trans_field = fruit.getField('translation')
                    fruit_trans_field.setSFVec3f(fruit_initial_translation)
                    if j == max and k == max: fr = 0
            i += 1
            if i == 120: i = 0
        
    pass

