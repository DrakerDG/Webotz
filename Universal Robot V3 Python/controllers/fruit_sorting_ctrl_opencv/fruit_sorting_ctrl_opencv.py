"""fruit_sorting_ctrl_opencv controller."""
import cv2
import numpy as np
from controller import Supervisor

robot = Supervisor()

# Set the time step in 32
timestep = 32

# Type of fruit:  0 = orange, 1 = apple
fruit = -1

# Fruits counters
orange = 0
apple = 0

# Delay counter
counter = 0

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

# Initialize speaker
speaker = robot.getDevice('speaker')

def playSnd(track):
    if track == 0: speaker.playSound(speaker, speaker, 'sounds/women/orange.wav', 1.0, 1.0, 0.0, False)
    elif track == 1: speaker.playSound(speaker, speaker, 'sounds/women/apple.wav', 1.0, 1.0, 0.0, False)

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
    roi = img[0:150, 35:165]
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
                printDisplay(x + 35, y, w, h, fname[i])

    """ Test images
    cv2.imwrite('00Roi.png', roi)
    cv2.imwrite('01HSV.png', imHSV)
    cv2.imwrite('02Orange.png', mask[0])
    cv2.imwrite('03Green.png', mask[1])
    """

    return model

# Main loop:
while robot.step(timestep) != -1:
    # ifs for the different state of the arm
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

    strP = f'Apples: {apple:3d}    Oranges: {orange:3d}'
    robot.setLabel(1, strP, 0.3, 0.96, 0.06, 0x00ff00, 0, 'Lucida Console')

    pass

