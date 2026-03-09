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

# possible colors
colors = [
    "1 0 0",      # red
    "1 1 0",      # yallow
    "1 0.5 0",    # orange
    "0 1 0"       # green
]

# parameters
NUM_OBSTACLES = 100
MIN_WIDTH = 0.3
MAX_WIDTH = 0.6
STEP = 0.2
X_RAND = 0.2

# generate winding path
path_points = []

x = robot_x - STEP
y = robot_y
z = 0.05


for i in range(NUM_OBSTACLES):

    x -= STEP
    y += random.uniform(-0.2,0.2)

    path_points.append((x,y))

# create obstacles
for i in range(NUM_OBSTACLES):

    p = random.choice(path_points)

    side = random.choice([-1,1])

    x = p[0] + random.uniform(-X_RAND,X_RAND)
    y = p[1] + side * random.uniform(MIN_WIDTH,MAX_WIDTH)

    if abs(y - robot_y) > MAX_WIDTH:
        continue
        
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
        }}
        """
    else:

        node = f"""
        Solid {{
          translation  {x} {y} {z}
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
        }}
        """

    children.importMFNodeFromString(-1,node)

while supervisor.step(timestep) != -1:
    pass


# Main loop:
while robot.step(timestep) != -1:
    pass

