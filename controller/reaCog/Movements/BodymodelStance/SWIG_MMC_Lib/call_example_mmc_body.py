import time
import matplotlib.pylab as py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from mmcBody import MmcBody
test_ext = MmcBody(0.2)

#before = time.time()
#test_ext.call_steps()
#after = time.time()
#print("TIME EXTERN: ", (after-before))
#c_time = after-before

#for i in range(0,4):
#	print(i, " - timestep ###################################")
#	mmcBM.mmc_iteration_step()
#	test_ext.mmc_iteration_step() 
#print("Speed up: ", (py_time/c_time))

for i in range(0, 100):
	test_ext.mmc_iteration_step()
print(test_ext.get_leg_vector(1))
import numpy
np_arr = numpy.array( test_ext.get_leg_vector(1) )
np_arr = numpy.array( [3., 2., 1., 0.] )
print(np_arr + test_ext.get_leg_vector(1))
test_ext.put_leg_on_ground(1, np_arr)