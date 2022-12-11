"""arm_ctrl_p controller."""

from controller import Robot
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Status
status = 0

# u, v 2D mouse position
u_now = 0
u_old = 0
v_now = 0
v_old = 0

# u, v factors & max value
u_fac = 0
v_fac = 0
max_v = 4

# Keyboard 
key = -1

# Initialize Motors
waist_motor = robot.getDevice('waist_motor')
waist_motor.setPosition(float('inf'))
waist_s = 1
waist_d = 0
waist_motor.setVelocity(waist_s * waist_d)

shoulder_motor = robot.getDevice('shoulder_motor')
shoulder_motor.setPosition(float('inf'))
shoulder_s = 1
shoulder_d = 0
shoulder_motor.setVelocity(shoulder_s * shoulder_d)

elbow_motor = robot.getDevice('elbow_motor')
elbow_motor.setPosition(float('inf'))
elbow_s = 1
elbow_d = 0
elbow_motor.setVelocity(elbow_s * elbow_d)

wrist_motor = robot.getDevice('wrist_motor')
wrist_motor.setPosition(float('inf'))
wrist_s = 1
wrist_d = 0
wrist_motor.setVelocity(wrist_s * wrist_d)

pitch_motor = robot.getDevice('pitch_motor')
pitch_motor.setPosition(float('inf'))
pitch_s = 1
pitch_d = 0
pitch_motor.setVelocity(pitch_s * pitch_d)

claw_motor = robot.getDevice('phalanx_motor::right')
claw_motor.setPosition(float('inf'))
claw_s = 1
claw_d = 0
claw_motor.setVelocity(claw_s * claw_d)

# Initialize Sensors
waist_p = robot.getDevice('waist_sensor')
waist_p.enable(timestep)

shoulder_p = robot.getDevice('shoulder_sensor')
shoulder_p.enable(timestep)

elbow_p = robot.getDevice('elbow_sensor')
elbow_p.enable(timestep)

wrist_p = robot.getDevice('wrist_sensor')
wrist_p.enable(timestep)

pitch_p = robot.getDevice('pitch_sensor')
pitch_p.enable(timestep)

fingers_p = robot.getDevice('phalanx_sensor')
fingers_p.enable(timestep)

finger_s = robot.getDevice('finger_sensor')
finger_s.enable(timestep)

# camera enable
camera = robot.getDevice('camera')
camera.enable(timestep*2)

# keyboard enable
robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

# mouse enable
robot.mouse.enable(timestep)
robot.mouse.enable3dPosition()

# Main loop:
while robot.step(timestep) != -1:

    # Get sensors value
    wa_p = waist_p.getValue()
    sh_p = shoulder_p.getValue()
    el_p = elbow_p.getValue()
    wr_p = wrist_p.getValue()
    pi_p = pitch_p.getValue()
    fi_p = fingers_p.getValue()
    fi_s = finger_s.getValue()
    
    # Get pressed key
    key = robot.keyboard.getKey()

    # Get mouse state
    mouse_state = robot.mouse.getState()
    
    # u, v factor estimation
    if mouse_state.left == 0 and mouse_state.middle == 0 and mouse_state.right == 0:
        u_now = mouse_state.u
        v_now = mouse_state.v
        if math.isnan(u_now):
            u_now = 0
            v_now = 0
            
        if u_now != u_old:
            u_fac = 50 * (u_now - u_old)
            if u_fac > max_v:
                u_fac = max_v
            if u_fac < -max_v:
                u_fac = -max_v
            u_old = u_now
        else:
            u_fac = 0
        if v_now != v_old:
            v_fac = 50 * (v_now - v_old)
            if v_fac > max_v:
                v_fac = max_v
            if v_fac < -max_v:
                v_fac = -max_v
            v_old = v_now
        else:
            v_fac = 0
    else:
        u_old = u_now
        v_old = v_now
        
    # Reset Status
    if status == 0:
        if sh_p > -2.181662:
            shoulder_d = -1
        else:
            shoulder_d = 0
            status = 1

        if el_p < 0.663225:
            elbow_d = 1
        else:
            elbow_d = 0        

        if fi_p < 1.029744:
            claw_d = 1
        else:
            claw_d = 0

    # Manual Status
    if status == 1:

        # z key waist & shoulder
        if key == 90:
            waist_d = -u_fac
            shoulder_d = -v_fac
            if (waist_d > 0 and wa_p > 3.141593) or (waist_d < 0 and wa_p < -3.141593):
                waist_d = 0
            if (shoulder_d > 0 and sh_p > 2.181662) or (shoulder_d < 0 and sh_p < -2.181662):
                shoulder_d = 0
        else:
             waist_d = 0
             shoulder_d = 0

        # x key elbow
        if key == 88:
            elbow_d = -v_fac
            if (elbow_d > 0 and el_p > 2.181662) or (elbow_d < 0 and el_p < -2.181662):
                elbow_d = 0
        else:
             elbow_d = 0
       
        # s key wrist
        if key == 83:
            wrist_d = u_fac
            if (wrist_d > 0 and wr_p > 3.141593) or (wrist_d < 0 and wr_p < -3.141593):
                wrist_d = 0
        else:
             wrist_d = 0
             
        # d key pitch
        if key == 68:
            pitch_d = -v_fac
            if (pitch_d > 0 and pi_p > 1.570796) or (pitch_d < 0 and pi_p < -1.570796):
                pitch_d = 0
        else:
             pitch_d = 0
             
        # c key fingers
        if key == 67:
            claw_d = v_fac
            if (claw_d > 0 and fi_p > 1.029744) or (claw_d < 0 and (fi_p < -0.200713 or fi_s == 1)):
                claw_d = 0
        else:
             claw_d = 0
     
    # Setting velocity of every motor        
    waist_motor.setVelocity(waist_s * waist_d)
    shoulder_motor.setVelocity(shoulder_s * shoulder_d)
    elbow_motor.setVelocity(elbow_s * elbow_d)
    wrist_motor.setVelocity(wrist_s * wrist_d)
    pitch_motor.setVelocity(pitch_s * pitch_d)
    claw_motor.setVelocity(claw_s * claw_d)

    pass

