import time

from mmcLeg import MmcLeg
test_ext = MmcLeg([0.06,0.3,0.26], 1)

before = time.time()
import numpy 
targ_np = numpy.array([0.32749283, -0.0114609, -0.01876802, 0.])
print(targ_np)
for i in range(0,10):
	test_ext.compute_inverse_kinematics(targ_np)
#test_ext.call_steps(0.1,0.0,0.0)
after = time.time()
print("TIME EXTERN: ", (after-before))