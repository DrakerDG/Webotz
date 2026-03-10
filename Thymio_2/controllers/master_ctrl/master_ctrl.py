"""master_ctrl controller."""

from controller import Supervisor
import random
import math

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# get robot
thymio = supervisor.getFromDef("Thymio2")
trans_field = thymio.getField("translation")
pos = trans_field.getSFVec3f()

robot_x = pos[0]
robot_y = pos[1]

root = supervisor.getRoot()
children = root.getField("children")

goal = supervisor.getDevice('goal')
goal.enable(timestep)

# possible colors
colors = [
    "1 0 0",      # red
    "1 1 0",      # yallow
    "1 0.5 0",    # orange
    "0 1 0"       # green
]

# parameters
NUM_OBSTACLES =  20
X_DISTANCE    = 4.0
STEP          = 0.6
X_RAND        = 0.3
Y_RAND        = 0.6
MARGIN        = 0.4

X_STAGES  = int(X_DISTANCE / STEP)
Y_SAMPLES = int(NUM_OBSTACLES / X_STAGES)

robot_x -= MARGIN
z  = 0.05

n = 0

tmr1 = 0
tmr2 = 0
run  = 1


for i in range(X_STAGES):

    for j in range(Y_SAMPLES):
        x = robot_x - STEP * i + random.uniform(-X_RAND, X_RAND)
        y = robot_y + random.uniform(-Y_RAND, Y_RAND)
    
        # create obstacles
        shape = random.choice(["box","cylinder"])
        color = random.choice(colors)

        if shape == "box":
            
            node = f"""
            Solid {{
              translation {x} {y} {z}
              children [
                Shape {{
                  appearance Appearance {{
                    material Material {{
                      diffuseColor {color}
                    }}
                  }}
                  geometry Box {{
                    size 0.1 0.1 0.1
                  }}
                }}
              ]
              name "obstacle{n}"
              boundingObject Box {{
                size 0.1 0.1 0.1
              }}
              physics Physics {{ }}
            }}
            """
        else:
    
            node = f"""
            Solid {{
              translation {x} {y} {z}
              children [
                Shape {{
                  appearance Appearance {{
                    material Material {{
                      diffuseColor {color}
                    }}
                  }}
                  geometry Cylinder {{
                    radius 0.05
                    height 0.1
                  }}
                }}
              ]
              name "obstacle{n}"
              boundingObject Cylinder {{
                    radius 0.05
                    height 0.1
              }}
              physics Physics {{ }}
            }}
            """
    
        children.importMFNodeFromString(-1,node)
        
        n += 1

# hour:minutes:seconds.thousandths
def hms(sec):
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 3600 % 60)
    c = (sec - int(sec)) * 1000
    tm = f'{h:02d}:{m:02d}:{s:02d}.{int(c):03d}'
    return tm

# print the status in simulatoin screen
def printStatus():
    global tmr0
    global tmr1
    
    if run == 1:
        tmr0 = supervisor.getTime()
        strP = hms(tmr0)
        strP = f'Lap Time:   {strP:s}'
        supervisor.setLabel(0, strP, 0, 0.89, 0.1, 0x00FF00, 0, 'Lucida Console')

    elif run == 0:
        tmr1 = supervisor.getTime() - tmr0
        if tmr1 > 6:
            run == 1
            supervisor.simulationSetMode(0)

# Main loop:    
while supervisor.step(timestep) != -1:

    goal_value = goal.getValue()
    
    if goal_value < 1000: run = 0
    
    printStatus()
    
    pass
