# Thymio2_ctrl controller
# V2.0
#        ____             __             ____  ______
#       / __ \_________ _/ /_____  _____/ __ \/ ____/
#      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
#     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
#    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
#

from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MAX_VEL = 9.53

# motors
leftMotor = robot.getDevice("motor.left")
rightMotor = robot.getDevice("motor.right")

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# compass
compass = robot.getDevice("compass")
compass.enable(timestep)

# sensors
names = ['prox.horizontal.0','prox.horizontal.1','prox.horizontal.2',
         'prox.horizontal.3','prox.horizontal.4']

sensors = []

for n in names:
    s = robot.getDevice(n)
    s.enable(timestep)
    sensors.append(s)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# parameters
forward = MAX_VEL
K_heading = 4 #0.05
K_avoid = 3 #0.002

escape_direction = 0
escape_timer = 0
escape_factor = 200
max_timer = 40
max_front = 500
max_delta = 2000
min_delta = 1500


def get_heading():
    v = compass.getValues()
    angle = math.atan2(v[1], v[0])
    return angle

desired_heading = 0.0  # north

# Main loop:
while robot.step(timestep) != -1:
    # orientation
    heading = get_heading()
    error = desired_heading - heading

    heading_correction = K_heading * error

    # sensors
    values = [s.getValue() for s in sensors]
    
    front_obs = values[1] + values[2] + values[3]
    left_obs  = values[0] + values[1]
    right_obs = values[3] + values[4]

    # print sensor values debug
    #print(f"front_obs: {front_obs: 10.2f}    delta: {abs(left_obs - right_obs): 10.2f} ")
    
    # Detects possible blockages or obstacles on both sides
    if (front_obs > max_front and abs(left_obs - right_obs) < max_delta) or front_obs == 0 and abs(left_obs - right_obs) > min_delta:

        if escape_timer == 0:
            import random
            escape_direction = random.choice([-1,1])
            escape_timer = max_timer
    
    if escape_timer > 0:
        avoidance = escape_direction * escape_factor
        escape_timer -= 1
    else:
        avoidance = K_avoid * (left_obs - right_obs)

    # velocityes
    vel_left  = forward + heading_correction + avoidance
    vel_right = forward - heading_correction - avoidance
    

    vel_left  = max(min(vel_left, MAX_VEL), -MAX_VEL)
    vel_right = max(min(vel_right, MAX_VEL), -MAX_VEL)
    
    # print speed values debug
    #print(f"left: {vel_left: 8.2f}   right: {vel_right: 8.2f}   heading: {heading: 8.2f}")

    leftMotor.setVelocity(vel_left)
    rightMotor.setVelocity(vel_right)

    pass
