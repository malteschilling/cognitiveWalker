import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

#import pycgtypes.vec3 as vec3
#import pycgtypes.quat as quat
import math, os

import controller.reaCog.WalknetSettings as WSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

##
#   Collect Data for a single experiment
##
class DataCollector(ProcessingModule):
    
    def __init__ (self, name, mot_net, imu, experiment_name, duration):
        ProcessingModule.__init__(self, name)
        self.motivationNet = mot_net
        self.imu = imu
        self.draw_counter = 0 
        
        self.data_points = duration*100 + self.draw_counter # Number of data points
#        self.data_array = #np.empty([self.data_points,23])#[None]*self.data_points
        
        self.experiment_name = experiment_name
        
    ##
    #   Init figure and subplots.
    def init_module(self):
        # FOOTFALL PATTERN
        self.leg_order = [0,3,1,4,2,5]
        self.lastswing = [-1,-1,-1,-1,-1,-1]
        
        import uuid
        self.file_name = "logs/reaCog_walking_" + self.experiment_name #+ str(uuid.uuid4())
        if os.path.isfile(self.file_name):
            os.remove(self.file_name)
        print("DATA COLLECTOR NAME: ", self.file_name)
        #self.f_handle = open("reaCog_walking_all_data.out", 'a')
        #> savetxt(f_handle, my_matrix)
        #self.file = open("reaCog_walking_all_data.out", "w")
#       self.ax_fig = plt.figure(figsize=(8, 6))
    
    ##
    #   Update visualization using data from the body model.    
    def post_processing_step(self, args=None):
        self.draw_counter+=1        
        # if (self.draw_counter == self.data_points):
#             # %f, %f, %f, %f, %f, %f, 
#             with open("reaCog_walking_all_data.out",'ba') as f_handle:
#                 np.savetxt( f_handle, [self.data_array[-1]], fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')
#             
#             #np.savetxt( str(self.experiment_name) + '_all_data.out', self.data_array, fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')
#             print("Wrote experiment data " + str(self.experiment_name))
#             #input()
        
        if (self.draw_counter >= 0):
            for j in range(0,6) :
                if self.motivationNet.motivationNetLegs[j].isStancing()==False and self.lastswing[j]<0 :
                    self.lastswing[j] = self.draw_counter
                elif self.motivationNet.motivationNetLegs[j].isStancing() and self.lastswing[j]>=0:     
                    self.lastswing[j]=-1
            if self.draw_counter < self.data_points:
                if (self.imu):
                    self.data_array = [self.imu.position[0], self.imu.position[1], self.imu.position[2],
                        self.motivationNet.motivationNetLegs[0].isStancing(), self.motivationNet.motivationNetLegs[1].isStancing(), self.motivationNet.motivationNetLegs[2].isStancing(),
                        self.motivationNet.motivationNetLegs[3].isStancing(), self.motivationNet.motivationNetLegs[4].isStancing(), self.motivationNet.motivationNetLegs[5].isStancing(),
                        self.motivationNet.cognitive_expansion.Phase.getCurrentPhase(),
                        self.motivationNet.motivationNetLegs[0].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[1].wleg.leg.input_foot_position[0], 
                        self.motivationNet.motivationNetLegs[2].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[3].wleg.leg.input_foot_position[0], 
                        self.motivationNet.motivationNetLegs[4].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[5].wleg.leg.input_foot_position[0],
                        self.motivationNet.bodyModelStance.temp_stability_fact_back, self.motivationNet.bodyModelStance.temp_stability_fact_front,
                        self.motivationNet.motivationNetLegs[0].pep_shifted[0], self.motivationNet.motivationNetLegs[1].pep_shifted[0],
                        self.motivationNet.motivationNetLegs[2].pep_shifted[0], self.motivationNet.motivationNetLegs[3].pep_shifted[0],
                        self.motivationNet.motivationNetLegs[4].pep_shifted[0], self.motivationNet.motivationNetLegs[5].pep_shifted[0] ]
                    for i in  range(0,6):
                        self.data_array.extend(self.motivationNet.motivationNetLegs[i].wleg.leg.getInputAngles())
                        self.data_array.extend([joint.torsion for joint in self.motivationNet.motivationNetLegs[i].wleg.leg.joints])
                    # Added information on phase relationships between neighboring leg controllers (1.7.)
                    self.data_array.extend(self.motivationNet.compareCurrentLegPhaseRelationsWithTripod())
                else:
                    self.data_array = [0.0, 0.0, 0.0,
                        self.motivationNet.motivationNetLegs[0].isStancing(), self.motivationNet.motivationNetLegs[1].isStancing(), self.motivationNet.motivationNetLegs[2].isStancing(),
                        self.motivationNet.motivationNetLegs[3].isStancing(), self.motivationNet.motivationNetLegs[4].isStancing(), self.motivationNet.motivationNetLegs[5].isStancing(),
                        self.motivationNet.cognitive_expansion.Phase.getCurrentPhase(),
                        self.motivationNet.motivationNetLegs[0].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[1].wleg.leg.input_foot_position[0], 
                        self.motivationNet.motivationNetLegs[2].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[3].wleg.leg.input_foot_position[0], 
                        self.motivationNet.motivationNetLegs[4].wleg.leg.input_foot_position[0], self.motivationNet.motivationNetLegs[5].wleg.leg.input_foot_position[0],
                        self.motivationNet.bodyModelStance.temp_stability_fact,
                        self.motivationNet.motivationNetLegs[0].pep_shifted[0], self.motivationNet.motivationNetLegs[1].pep_shifted[0],
                        self.motivationNet.motivationNetLegs[2].pep_shifted[0], self.motivationNet.motivationNetLegs[3].pep_shifted[0],
                        self.motivationNet.motivationNetLegs[4].pep_shifted[0], self.motivationNet.motivationNetLegs[5].pep_shifted[0] ]
                    for i in  range(0,6):
                        self.data_array.extend(self.motivationNet.motivationNetLegs[i].wleg.leg.getInputAngles())
                        self.data_array.extend([joint.torsion for joint in self.motivationNet.motivationNetLegs[i].wleg.leg.joints])
                #self.z_array[self.draw_counter] = self.imu.position[1] #self.fx[self.draw_counter]
            #np.savetxt(self.f_handle, "NP ADD")
            #np.savetxt("reaCog_walking_all_data.out", self.data_array, fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')
            #print("SAVED DATA")
            with open( self.file_name,'ba') as f_handle:
                np.savetxt( f_handle, np.atleast_2d(self.data_array), fmt='%1.3f') 
                #np.savetxt( f_handle, [self.data_array[self.draw_counter]], fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')
#            with open("reaCog_walking_all_data.out",'ba') as f_handle:
 #               np.savetxt( f_handle, self.data_array, fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')
            #self.file.write("\n") 