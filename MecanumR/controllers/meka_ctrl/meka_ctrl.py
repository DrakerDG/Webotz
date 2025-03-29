"""meka_ctrl controller."""

from cfv_gamepad import cfv_gamepad    # Import the configuration gamepad library
from controller import Supervisor      # Import controller as Supervisor

## Initialize the robot
robot = Supervisor()

# Get self as robot node
robotNode = robot.getSelf()

# Get basic timestep
timestep = int(robot.getBasicTimeStep())

# Create a joystick object
joy = cfv_gamepad()
joy_max = 32768  # Joystick max value to calibration
joystick = 1     # Enable joystick

# Robot dimmensions
r = 0.05  # wheel radius
a, b = 0.12, 0.12  # axle distances

# Linear and angular velocities
vx, vy, omega_r = 0, 0, 0
linear_speed = 0.4
angular_speed = 1.4
max_linear_speed = 0.5
max_angular_speed = 1.5

# Timer to the LEDs
timer = 0

# Initialize motors and sensors
motors = []

for i in range(4):
    motor = robot.getDevice('motor' + str(i))
    motors.append(motor)
    motors[i].setPosition(float('inf'))
    motors[i].setVelocity(0)

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep*4)

# Initialize LEDs
leds = []

for i in range(2):
    led = robot.getDevice('led' + str(i))
    leds.append(led)
    leds[i].set(1)
    
# Hour:minutes:seconds.thousandths
def hms(sec):
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 3600 % 60)
    c = (sec - int(sec)) * 1000
    tm = f'{h:02d}:{m:02d}:{s:02d}.{int(c):03d}'
    return tm

# Calculation of inverse kinematics
def calc_IK(robot_vel):
    vx, vy, omega_r = robot_vel[0], robot_vel[1], robot_vel[2]

    omega_1 = 1/r * (vx - vy - (a + b) * omega_r)
    omega_2 = 1/r * (vx + vy + (a + b) * omega_r)
    omega_3 = 1/r * (vx + vy - (a + b) * omega_r)
    omega_4 = 1/r * (vx - vy + (a + b) * omega_r)
    
    return [omega_1, omega_2, omega_3, omega_4]

#Calculation of direct kinematics
def calc_DK(motor_vel):
    w1, w2, w3, w4 = motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]

    vx = r * (w1 + w2 + w3 + w4)/4
    vy = r * (- w1 + w2 + w3 - w4)/4
    omega_r = r * (- w1 + w2 - w3 + w4)/(4*(a + b))

    return [vx, vy, omega_r]

# Main loop:
while robot.step(timestep) != -1:
    
    if joystick == 1:  # If joystick is connected
        
        # Set x factor
        vx = round(-(joy.LeftJoystickY + joy_max)/joy_max + 1, 1)

        # Set y factor
        vy = round(joy.RightJoystickX/1023, 0)
        if abs(vy) < 0.1: vy = round(-joy.RightJoystickY/1023, 0)
        
        # Set omega factor
        omega_r = round(-(joy.LeftJoystickX + joy_max)/joy_max + 1, 1)
        
        speed_up = bool(joy.B)    # speed increase indicator
        speed_down = bool(joy.A)  # speed decrease indicator
        
        # Increase speed
        if speed_up:
            if abs(linear_speed) <= max_linear_speed: linear_speed *= 1.01
            if abs(angular_speed) <= max_angular_speed: angular_speed *= 1.01
        
        # Decrease speed
        if speed_down:
            if abs(linear_speed) >= 0.01: linear_speed *= 0.99
            if abs(angular_speed) >= 0.01: angular_speed *= 0.99            
        
        # Exit simulation
        if joy.Y == 1:
            robot.simulationSetMode(0)
            robot.worldReload()
        
        # Calculation of speeds
        robot_vel = [     
            round(vx * linear_speed, 2),
            round(vy * linear_speed, 2),
            round(omega_r * angular_speed, 2)
            ]
        
        # Calculation of angular velocities by inverse kinematics
        motor_vel = calc_IK(robot_vel)
        
        # Sets the speeds of each motor
        for i in range(4):
            motors[i].setVelocity(motor_vel[i])
        
        # Display timer
        tmr = robot.getTime()
        strP = hms(tmr)
        strP = f'Timer:   {strP:s}'
        robot.setLabel(0, strP, 0, 0.85, 0.06, 0x000000, 0, 'Lucida Console')
        
        # Display robot name
        strP = f'Robot:   {robot.getName():s}'
        robot.setLabel(1, strP, 0, 0.89, 0.06, 0x000000, 0, 'Lucida Console')
        
        # Display robot velocities
        strP = f'vx: {robot_vel[0]:6.2f} m/s      vy: {robot_vel[1]:6.2f} m/s      omega_r: {robot_vel[2]:6.2f} rad/s'
        robot.setLabel(2, strP, 0, 0.93, 0.06, 0x000000, 0, 'Lucida Console')
        
        # Display wheels velocities
        strP = f'w1: {motor_vel[0]:6.2f} rad/s    w2: {motor_vel[1]:6.2f} rad/s    w3: {motor_vel[2]:6.2f} rad/s    w4: {motor_vel[3]:6.2f} rad/s'
        robot.setLabel(3, strP, 0, 0.97, 0.06, 0x000000, 0, 'Lucida Console')
        
        # Blinking LEDs
        if timer > 30:
            for i in range(2):
                if leds[i].get() == 0:
                    leds[i].set(1)
                else:
                    leds[i].set(0)
            timer = 0

        timer += 1
  
    pass
