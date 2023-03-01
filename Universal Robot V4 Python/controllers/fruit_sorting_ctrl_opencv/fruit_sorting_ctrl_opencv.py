"""fruit_sorting_ctrl_opencv controller
    Sample code in Python
      ** Using UR5e from Universal Robot
      ** Using the opencv library to detect each fruit (installation required)
      ** Using the numpy library to handle the array (installation required)
      ** Using the os library to get the path to save display images (HMI)
"""

import cv2
import numpy as np
import os

from controller import Supervisor
from controller import Mouse

robot = Supervisor()

# Set the time step in 32
timestep = 32

# Type of fruit:  0 = orange, 1 = apple
fruit = -1

# Fruits counters
orange = 0
apple = 0

# Poweron conveyor belt
pwr_cb = False
click = False

# Delay counter
counter = 0
timer = 0

# Status of UR5e Robot
state = 0

# Target positions to drop fruits
target_positions = [-1.570796, -1.87972, -2.139774, -2.363176, -1.50971]

# Speed of UR5e Robot
speed = 2

# Getting and declaring the 3 finger motors of the gripper 
hand_motors = []
hand_motors.append(robot.getDevice('finger_1_joint_1'))
hand_motors.append(robot.getDevice('finger_2_joint_1'))
hand_motors.append(robot.getDevice('finger_middle_joint_1'))

# Getting and declaring the robot motor
ur_motors = []
ur_motors.append(robot.getDevice('shoulder_pan_joint'))
ur_motors.append(robot.getDevice('shoulder_lift_joint'))
ur_motors.append(robot.getDevice('elbow_joint'))
ur_motors.append(robot.getDevice('wrist_1_joint'))
ur_motors.append(robot.getDevice('wrist_2_joint'))

# Seting velocity of UR5e motors
for i in range(5):
    ur_motors[i].setVelocity(speed)

# Getting and declaring distance sensor of gripper
distance_sensor = robot.getDevice('distance sensor')
distance_sensor.enable(timestep)

# Getting and declaring position sensor of wrist robot
position_sensor = robot.getDevice('wrist_1_joint_sensor')
position_sensor.enable(timestep)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Initialize display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

# Initialize HMI
hmi = robot.getDevice('hmi')
hmi.setColor(0x000000)
hmi.setFont('Verdana', 14, True)

# Initialize speaker
speaker = robot.getDevice('speaker')

# mouse enable
robot.mouse.enable(timestep)
#robot.mouse.enable3dPosition()

# Getting the field of the conveyor belt speed
cb = robot.getFromDef('cb')
cb_field = cb.getField('speed')

# Getting the field of the control panel url
cp = robot.getFromDef('CP')
cp_field = cp.getField('url')

id = 0

def convertPath(path):
    separator = os.path.sep
    if separator != '/':
        path = path.replace(os.path.sep, '/')
    return path

def getFilePath(file):
    path0 = os.path.abspath(os.getcwd())
    path0 = convertPath(path0)
    find0 = path0.find('controllers')
    path0 = path0[:find0]
    file0 = path0 + 'worlds/' + 'textures/' + file
    return file0

def playSnd(track):
    if track == 0: speaker.playSound(speaker, speaker, 'sounds/women/orange.wav', 1.0, 1.0, 0.0, False)
    elif track == 1: speaker.playSound(speaker, speaker, 'sounds/women/apple.wav', 1.0, 1.0, 0.0, False)

def ctrl_HMI():
    global id
    
    # Refresh display of HMI
    hmi.setAlpha(0.0)
    hmi.fillRectangle(0, 0, 200, 150)
    hmi.setAlpha(1.0)

    # get base image
    img = cv2.imread('hmi.png')

    # writing the amount of oranges in the image
    strf = f'Oranges: {orange:3d}'
    cv2.putText(img, strf, (50, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)

    # writing the amount of apples in the image
    strf = f'  Apples: {apple:3d}'
    cv2.putText(img, strf, (50, 128), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)

    # writing the power status and time (seconds) in the image
    if pwr_cb:
        cb_field.setSFFloat(0.15)
        pwr = 'ON '
    else:
        cb_field.setSFFloat(0.0)
        pwr = 'OFF'
    strf = f'{pwr:s} time: {robot.time:3.0f}'
    cv2.putText(img, strf, (45, 32), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)

    # making copy of image (256,256) to display panel (render)
    rsz = cv2.resize(img, (256,256), fx=0, fy= 0, interpolation = cv2.INTER_CUBIC)

    # save images
    cv2.imwrite('hmi00' + str(id) + '.png', img)
    cv2.imwrite(getFilePath('hmi00' + str(id) + '.jpg'), rsz)

    # loaging image to display
    hmi_image = hmi.imageLoad('hmi00' + str(id) + '.png')
    hmi.imagePaste(hmi_image, 0, 0, True)

    # loading image to diaplay panel (render)
    cp_field.setMFString(0, 'textures/hmi00' + str(id) + '.jpg')
    if id == 0: id = 1
    else: id = 0
            
def resetDisplay():
    display.setAlpha(0.0)
    display.fillRectangle(0, 0, 200, 150)
    display.setAlpha(1.0)

def printDisplay(x, y, w, h, name):
    resetDisplay()
    display.drawRectangle(x, y, w, h)
    display.drawText(name, x - 2, y - 20)

def findFruit():
    min = []
    max = []
    cnts = []
    mask = []
    model = -1

    # Fruit names
    fname = ['Orange', 'Apple']

    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    roi = img[10:150, 35:165]
    imHSV = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Orange HSV color range
    min.append(np.array([10, 135, 135], np.uint8))
    max.append(np.array([32, 255, 255], np.uint8))
    
    # Green HSV color range
    min.append(np.array([30, 50, 50], np.uint8))
    max.append(np.array([90, 255, 255], np.uint8))
    
    # Kernel filter 
    Kernel = np.ones((5, 5), np.uint8)
    
    for i in range(2):
        mask.append(cv2.inRange(imHSV, min[i], max[i]))
        
        # Morphological transformations with orange mask and kernel
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_CLOSE, Kernel)
        mask[i] = cv2.morphologyEx(mask[i], cv2.MORPH_OPEN, Kernel)

        # Finding contours
        cnts.append(cv2.findContours(mask[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0])

        for c in cnts[i]:
            x, y, w, h = cv2.boundingRect(c)
            if w > 80:
                model = i
                printDisplay(x + 35, y + 10, w, h, fname[i])

    return model

ctrl_HMI()

# Main loop:
while robot.step(timestep) != -1:
    
    # Get mouse state
    mouse_state = robot.mouse.getState()
    if mouse_state.v > 0.026 and mouse_state.v < 0.059 and mouse_state.u > 0.017 and mouse_state.u < 0.047:
        if mouse_state.left and not click:
            pwr_cb = not pwr_cb
            click = True
            
    if not mouse_state.left and click:
        click = False
    
    if counter <= 0:
        if state == 0: # WAITING
            fruit = findFruit()
            if distance_sensor.getValue() < 500:
                state = 1 # PICKING
                playSnd(fruit)
                if fruit == 0:
                    orange += 1
                elif fruit == 1:
                    apple += 1
                counter = 8
                for i in range(3):
                    hand_motors[i].setPosition(0.52)

        elif state == 1: # PICKING
            for i in range(fruit, 5):
                ur_motors[i].setPosition(target_positions[i])
            state = 2 # ROTATING

        elif state == 2: # ROTATING
            if position_sensor.getValue() < -2.3:
                counter = 8
                state = 3 # DROPPING
                resetDisplay()
                for i in range(3):
                    hand_motors[i].setPosition(hand_motors[i].getMinPosition())

        elif state == 3: # DROPPING
            for i in range(fruit, 5):
                ur_motors[i].setPosition(0.0)
            state = 4 # ROTATE_BACK

        elif state == 4: # ROTATE_BACK
            if position_sensor.getValue() > -0.1:
                state = 0 # WAITING
    else:
        counter -= 1

    if timer == 10:
        ctrl_HMI()
        timer = 0
    timer += 1
    
    pass

