## test PSM2 

import crtk, dvrk
ral = crtk.ral('test_PSM2')
p = dvrk.arm(ral, 'PSM2')
ral.check_connections()
ral.spin()
p.enable()
p.home()

# get measured joint state
[position, velocity, effort, time] = p.measured_js()
# get only position
position = p.measured_jp()
# get position and time
[position, time] = p.measured_jp(extra = True)

# move in joint space
import numpy
p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

# move in cartesian space
import PyKDL
# start position
goal = p.setpoint_cp()
# move 5cm in z direction
goal.p[2] += 0.05
p.move_cp(goal).wait()

import math
# start position
goal = p.setpoint_cp()
# rotate tool tip frame by 25 degrees
goal.M.DoRotX(math.pi * 0.25)
p.move_cp(goal).wait()

ral.shutdown()
