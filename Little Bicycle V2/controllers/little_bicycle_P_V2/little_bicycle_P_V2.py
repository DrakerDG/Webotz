"""little_bicycle_P_V2.1 controller."""

import cv2
import numpy as np
from controller import Supervisor

robot = Supervisor()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

robotNode = robot.getSelf()

preview = 0 # Mask Preview 1

Kp = 0.01
Ki = 0.02
Kd = 0.0001

P = 0
I = 0
D = 0
oldP = 0
PID = 0

# Bicycle speed
maxS = 6 # max speed
minS = 3 # min speed

# Handlebar angle
hMax = 0.1920 # rads (11°) Max
hndB = 0 # center
maxV = 0          # Max Velocity

# get wheel motor
whemotor = robot.getDevice('motor::crank')
whemotor.setPosition(float('inf'))
whemotor.setVelocity(maxS)

# get handlebars motor
hndmotor = robot.getDevice('handlebars motor')
hndmotor.setPosition(0)

# keyboard enable
robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep*4)

# Initialize display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)
display.setFont('Verdana', 16, True)

if preview == 1:
    cv2.startWindowThread()

def getError(act_error):
    error_P = act_error

    # get image form camera bicycle
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    # height and width of image
    h = img.shape[0]
    w = img.shape[1]

    # set point in x and y
    xSet = int(w / 2) - 10
    ySet = 205

    mask = np.zeros((h, w), dtype=np.uint8)
    pts = np.array([[[90, 200], [390, 200], [410, 210], [70, 210]]]) # Bicycle
    cv2.fillPoly(mask, pts, 255)
    zone = cv2.bitwise_and(img, img, mask=mask)
    
    hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
    
    dark_color = np.array([75, 0, 0])
    bght_color = np.array([179, 255, 255])
    Kernel = np.ones((5, 5), np.uint8)

    mask0 = cv2.inRange(hsv, dark_color, bght_color)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, Kernel)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, Kernel)

    if preview == 1:
        cv2.imshow("preview", mask0)
        cv2.waitKey(1)

    try:
        cnts0, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    except:
        return error_P
    else:
        try:
            largest_contour = max(cnts0, key=cv2.contourArea)
        except:
            return error_P
        else:
            try:
                x,y,w,h = cv2.boundingRect(largest_contour)
                center_x = int(x + w / 2)
                display.setAlpha(0.0)
                display.fillRectangle(0, 0, display.width, display.height)
                display.setAlpha(1.0)
                display.drawLine(center_x - 20, ySet, center_x + 20, ySet)
                display.drawLine(center_x, ySet - 20, center_x, ySet + 20)
                display.fillOval(center_x, ySet, 3, 3)
            except:
                return error_P
            else:
                return xSet - center_x 

def keyCtrl():
    # Get pressed key
    key=robot.keyboard.getKey()
    
    if key == 315: bcyS = maxS
    elif key < 0: bcyS = bcyS * 95 / 100

    if key == 314: hndB = hMax # left key
    elif key == 316: hndB = -hMax # right key
    else: hndB = 0 # center
    
    # Set velocity rear wheel
    whemotor.setVelocity(bcyS)

    # Set position handlebar
    hndmotor.setPosition(hndB)

# hour:minutes:seconds
def hms(sec):
    h = sec // 3600
    m = sec % 3600 // 60
    s = sec % 3600 % 60
    tm = f'{h:02d}:{m:02d}:{s:02d}'
    return tm


def printStatus():
    global maxV
    # Get Velocity
    velo = robotNode.getVelocity()
    
    # Velocity calulation:  Speed Module (x, y, z)
    velocity = (velo[0]**2 + velo[1]**2 + velo[2]**2)**0.5
    velocity = velocity * 3.6 # km/h
    
    if velocity > maxV:
        maxV = (velocity + maxV) / 2

    timer = int(robot.getTime())
    strP = hms(timer)
    
    if robot.getName() == 'Little Bicycle 1':
        vpos = 0.93
        strP = f'Time: {strP:s}'
        robot.setLabel(0, strP, 0, 0.97, 0.06, 0x000000, 0, 'Lucida Console')
    elif robot.getName() == 'Little Bicycle 2':
        vpos = 0.89
    strP = f'Robot: {robot.getName():s}   Speed: {velocity:5.2f} km/h   Max {maxV:5.2f} km/h'
    robot.setLabel(1, strP, 0, vpos, 0.06, 0x000000, 0, 'Lucida Console')
        
# Main loop:
while robot.step(timestep) != -1:

    P = getError(P)
    I = I * 2 / 3 + P * timestep / 1000
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    PID = Kp * P + Ki * I + Kd * D
    oldP = P

    hndB = hMax - abs(PID)
    hndB = hndB + PID
    if hndB > hMax: hndB = hMax
    elif hndB < -hMax: hndB = -hMax
    
    bcyS = maxS
    bcyS = bcyS - abs(PID * 4)
    if bcyS < minS: bcyS = minS

    # Set position handlebar
    hndmotor.setPosition(hndB)
    whemotor.setVelocity(bcyS)

    printStatus()
    
    pass
