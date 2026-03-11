# master_ctrl controller
# V2.0
#        ____             __             ____  ______
#       / __ \_________ _/ /_____  _____/ __ \/ ____/
#      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
#     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
#    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
#

from controller import Supervisor
import random
import math

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# get robot
thymio = supervisor.getFromDef("Thymio2")
thymio_field = thymio.getField("translation")
pos = thymio_field.getSFVec3f()

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
Y_DISTANCE    = 4.0
STEP          = 0.6
X_RAND        = 0.6
Y_RAND        = 0.3
MARGIN        = 0.6

Y_STAGES  = int(Y_DISTANCE / STEP)
X_SAMPLES = int(NUM_OBSTACLES / Y_STAGES)

robot_y = MARGIN
x = 0
y = -20
z = 0.05
n = 0

# Test status
RESET = 0
RUN   = 1
GOAL  = 2
STOP = 3

# Initial state
state = RESET

tmr0  = 0
tmr1  = 0
tmr2  = 0
wait  = 8
lapse = 0

max_lapse    = 10
best_time    = 10000
worst_time   = 0
running_time = 0
average_time = 0



for i in range(Y_STAGES):

    for j in range(X_SAMPLES):
            
        # create obstacles
        shape = random.choice(["box","cylinder"])
        color = random.choice(colors)

        if shape == "box":
            
            node = f"""
            DEF obstacle{n} Solid {{
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
            DEF obstacle{n} Solid {{
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
        
        y += 0.2
        n += 1

# hour:minutes:seconds.thousandths
def hms(sec):
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 3600 % 60)
    c = (sec - int(sec)) * 1000
    tm = f'{h:02d}:{m:02d}:{s:02d}.{int(c):03d}'
    return tm

def random_position():
    
    n = 0
    for i in range(Y_STAGES):
    
        for j in range(X_SAMPLES):
            x = robot_x + random.uniform(-X_RAND, X_RAND)
            y = robot_y + STEP * i + random.uniform(-Y_RAND, Y_RAND)

            name = f'obstacle{n:d}'
            obstacle = supervisor.getFromDef(name)
            obstacle_field = obstacle.getField('translation')
            obstacle_field.setSFVec3f([x, y, z])
            n += 1

# print the status in simulatoin screen
def printStatus():
    global tmr1
    
    if state == RUN:
        strP = hms(tmr1)
        strP = f'Lap Time{lapse:3d}:   {strP:s}'
           
        supervisor.setLabel(0, strP, 0, 0.84, 0.07, 0x00FF00, 0, 'Lucida Console')

    elif state == GOAL:
        strP = hms(best_time)
        strP = f'       Best:   {strP:s}'
        supervisor.setLabel(1, strP, 0, 0.88, 0.07, 0x00FF00, 0, 'Lucida Console')
        strP = hms(worst_time)
        strP = f'      Worst:   {strP:s}'
        supervisor.setLabel(2, strP, 0, 0.92, 0.07, 0x00FF00, 0, 'Lucida Console')
        strP = hms(average_time)
        strP = f'    Average:   {strP:s}'
        supervisor.setLabel(3, strP, 0, 0.96, 0.07, 0x00FF00, 0, 'Lucida Console')


# Main loop:    
while supervisor.step(timestep) != -1:

    if state == RESET:
        thymio_field.setSFVec3f([0, 0, 0])
        random_position()
        tmr0 = supervisor.getTime()
        state = RUN
        lapse +=1
        
    elif state == RUN:
        goal_value = goal.getValue()
        tmr1 = supervisor.getTime() - tmr0
        if goal_value < 1000:
            tmr0 = supervisor.getTime()
            if tmr1 < best_time :  best_time = tmr1
            if tmr1 > worst_time: worst_time = tmr1
            running_time += tmr1
            average_time = running_time / lapse
            state = GOAL    

    elif state == GOAL:
        tmr2 = supervisor.getTime() - tmr0
        if tmr2 > wait:
            if lapse < max_lapse:
                state = RESET
            else:
                state = STOP

    elif state == STOP:
        supervisor.simulationSetMode(0)
        
    printStatus()
    
    pass

