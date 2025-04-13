"""ESP32-CAM01 controller."""

import cv2                         # Import opencv library                  
import numpy as np                 # Import numpy library
import math                        # Import math library
from controller import Supervisor  # Import controller as Supervisor


robot = Supervisor()          # Create the Robot instance
robotNode = robot.getSelf()   # Get self as robot node

timestep = int(robot.getBasicTimeStep())  # get the time step of the current world

preview = 0 # Mask Preview 1: on  0: off

#  PID keys
Kp = 0.03     # 0.03
Ki = 0.06     # 0.06
Kd = 0.0003   # 0.0003

P = 0         # Proportional Error
I = 0         # Integral Error
D = 0         # Derivative Error 

PID = 0       # PID value

oldP = 0      # Old Proportional Error
maxS = 120    # Max Speed (120)    
ks = -20      # Gearbox key
maxV = 0      # Max Velocity

st_angle = 0  # Steering angle

tmr_on = 0    # LEDs timer

speed = 0     # Base speed
fwd = [1, 1, 1, 1]  # Speed wheel factors
run = 0       # Run mode

# Initialize Wheel Motors
wheels = []
for i in range(4):
    wheel = robot.getDevice('wheel' + str(i))
    wheels.append(wheel)
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(speed * fwd[i])

# Initialize servo motors
F_servo = robot.getDevice('F_servo')
F_servo.setPosition(0)
B_servo = robot.getDevice('B_servo')
B_servo.setPosition(0)
cam_servo = robot.getDevice('cam_servo')
cam_servo.setPosition(0.6109)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep*2)

# Initialize display
display = robot.getDevice('display')
display.attachCamera(camera)
display.setColor(0x00FF00)

# Initialize LEDs
leds = []
for i in range(2):
    led = robot.getDevice('led' + str(i))
    leds.append(led)
    leds[i].set(1)

# Initialize speaker
speaker = robot.getDevice('speaker')

# Start the opencv preview windows
def cv2windows():
    cv2.startWindowThread()
    cv2.namedWindow("HSV image", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Kernel Mask", cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("HSV image", 0, 900)
    cv2.moveWindow("Kernel Mask", 1540, 900)

# hour:minutes:seconds.thousandths
def hms(sec):
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 3600 % 60)
    c = (sec - int(sec)) * 1000
    tm = f'{h:02d}:{m:02d}:{s:02d}.{int(c):03d}'
    return tm

# Get error position (Proportional Error)
def get_error(act_error):
    
    # Getting image (frame) from the camera
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    #Reducing the image to 80x60 pixels
    img0 = cv2.resize(img, (80, 60), fx=0, fy= 0, interpolation = cv2.INTER_CUBIC)
    
    # Inverting the image (negative)
    img0 = cv2.bitwise_not(img0)

    h = img0.shape[0]   # Image height
    w = img0.shape[1]   # Image width

    xSet = int(w /2)    # Horizontal image target (center)
    ySet = 30           # Vertical image objective
    
    # Generates an empty mask
    mask = np.zeros((h, w), dtype=np.uint8)
    
    # Generating target polygon array:
    # Top: 50 pixels    Bottom: 60 pixels   high: 5 pixels
    pts = np.array([[[15, 28], [65, 28], [70, 32], [10, 32]]])
    # Generates the target mask
    cv2.fillPoly(mask, pts, 255)

    # Mask the image with the target mask
    zone = cv2.bitwise_and(img0, img0, mask=mask)
    
    # Change the color format from RGB to HSV
    hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)

    # Dark base tone color
    dark_color = np.array([0, 0, 100])
    
    # Light base tone color
    bght_color = np.array([179, 255, 255])
    
    # Generation of 3x3 Kernel matrix
    Kernel = np.ones((3, 3), np.uint8)

    # Applying light and dark tone filter
    mask0 = cv2.inRange(hsv, dark_color, bght_color)
    
    # Applying Kernel filters to discriminate noise
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_CLOSE, Kernel)
    mask0 = cv2.morphologyEx(mask0, cv2.MORPH_OPEN, Kernel)

    # If preview is 1, show HSV windows and Kernel mask
    if preview == 1:
        cv2.imshow("HSV image", hsv)
        cv2.imshow("Kernel Mask", mask0)
        cv2.waitKey(1)
        
    try:
        # Find the largest segmented contour (black line) and it's center
        contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    except:
        # If it is not found, it returns the previous error
        return act_error
    else:
        try:
            # Select the largest outline
            largest_contour = max(contours, key=cv2.contourArea)
        except:
            # If it is not found, it returns the previous error
            return act_error
        else:
            try:
                # Determine the dimensions of the outline
                x,y,w,h = cv2.boundingRect(largest_contour)
                
                # Calculate the center in x of the contour
                center_x = int(x + w / 2)
                
                # Clean the display
                display.setAlpha(0.0)
                display.fillRectangle(0, 0, display.width, display.height)
                display.setAlpha(1.0)
                
                # Scale the x position to 2.5 for the display
                x0 = int(center_x * 2.5)
                
                # Scale the y position to 2.5 for the display
                y0 = int(ySet * 2.5)
                
                # Draw the target on the display
                display.drawLine(x0 - 15, y0, x0 + 15, y0)
                display.drawLine(x0, y0 - 15, x0, y0 + 15)
                display.drawOval(x0, y0, 6, 6)
            except:
                # If it is not found, it returns the previous error
                return act_error
            else:
                # Returns the error value in x
                return xSet - center_x


# Gearbox to determine base speed depending on spin (error)
def gearbox(error):
    # Calculate the slope value based on the error and the ks constant
    slope = ks  * error 
    
    # Limits the slope value
    if abs(slope) > 0.5 * maxS:
        slope = -0.5 * maxS
        
    # Use maximum speed as a base
    base = maxS
    
    # Determine the final base speed
    speed = int(base + slope)

    # Returns the final base speed
    return speed


# line Following Module
def LineFollowingModule():
    global P
    global I
    global D
    global PID
    global oldP
    global left_speed
    global right_speed
    global cw
    global fwd
    global ab
    
    # Proportional Error
    P = get_error(P)
    
    # Integral error with 2/3 damping
    I = I * (2 / 3) + P * timestep / 1000
    
    # Derivative Error
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    # PID Calculation
    PID = Kp * P + Ki * I + Kd * D
    
    # Save the value of P
    oldP = P

    # Use the absolute value of PID to determine the omega angle
    omega = abs(PID)
    
    # Limit the value of omega
    if omega > 0.6109: omega: 0.6109
    
    # Determine the final value of the base speed based on the omega angle
    speed = gearbox(omega)
    
    # If omega is greater than 0
    if omega > 0:

        # Determine the value of angle theta
        theta = math.atan(1 / (-3/2 + 1 / math.tan(omega)))
        
        # Limit the value of angle theta
        if theta <0: theta = 1.520537
        
        # If the deviation is greater than zero (PID Error)
        if PID > 0:
            # Determine the angle of rotation (right)
            s_angle = omega
            
            # Determine the speed factor of each motor
            fwd[0] = 1
            fwd[1] = 1
            fwd[2] = math.sin(omega) / math.sin(theta)
            fwd[3] = math.sin(omega) / math.sin(theta)

        # If the deviation is less than zero (PID Error)
        else:            
            # Determine the angle of rotation (Left)
            s_angle = -omega
            
            # Determine the speed factor of each motor
            fwd[0] = math.sin(omega) / math.sin(theta)
            fwd[1] = math.sin(omega) / math.sin(theta)
            fwd[2] = 1
            fwd[3] = 1

    else:
        # If the deviation is zero (PID Error), reset the speed factors and the angle of rotation
        fwd = [1, 1, 1, 1]
        s_angle = 0

    # Set the speed of each motor for each wheel    
    for i in range(4):
        wheels[i].setVelocity(speed * fwd[i])
    
    # Sets the rotation angle of each servo motor    
    F_servo.setPosition(s_angle)
    B_servo.setPosition(-s_angle)

# Procedure to display the robot status
def printStatus():
    global maxV
    global led_on
    global tmr_on
    global run
    
    # Get Velocity
    velo = robotNode.getVelocity()
    
    # Velocity calulation:  Speed Module (x, y, z)
    velocity = (velo[0]**2 + velo[1]**2 + velo[2]**2)**0.5
    
    # Determine if the speed is maximum
    if velocity > maxV:
        maxV = (velocity + maxV) / 2

    # Generates robot sound effect based on its speed
    speaker.playSound(speaker, speaker, 'sounds/motor_0.wav', 1.0, velocity, 0.0, True)
    
    # Get the simulation time
    tmr = robot.getTime()
    
    # Writes the time on the screen in h:m:s format
    strP = hms(tmr)
    strP = f'Timer:   {strP:s}'
    robot.setLabel(0, strP, 0, 0.89, 0.06, 0x00FF00, 0, 'Lucida Console')

    # Write the robot's name on the screen
    strP = f'Robot:   {robot.getName():s}'
    robot.setLabel(1, strP, 0, 0.93, 0.06, 0x00FF00, 0, 'Lucida Console')

    # Write the current and maximum speed of the robot on the screen
    strP = f'Speed: {velocity:7.3f} m/s   Max: {maxV:5.3f} m/s'
    robot.setLabel(2, strP, 0, 0.97, 0.06, 0x00FF00, 0, 'Lucida Console')

    # Routine to turn the robot's LEDs on and off
    if tmr_on == 25:
        tmr_on = 0
        for i in range(2):
            if leds[i].get() == 1:
                leds[i].set(0)
            else:
                leds[i].set(1)
    tmr_on += 1

    # Pause before starting the track
    if int(tmr * 100) == 100:
        run = 1
    if run == 1:
        if preview == 1:
            cv2windows()
        run = 2
    
# Main loop:
while robot.step(timestep) != -1:

    # If the robot is in Run mode equal to 2, it starts the run on the track
    if run == 2:
        LineFollowingModule()
    
    # Print the robot status on the screen
    printStatus()

    pass

