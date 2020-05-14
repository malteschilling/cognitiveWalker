import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

#import pycgtypes.vec3 as vec3
#import pycgtypes.quat as quat
import math

import controller.reaCog.WalknetSettings as WSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule
from controller.reaCog.MotivationNetwork.MotivationUnit import getGroupsFromNames

##
#   Collect Data for a single experiment
##
class DataCollectorMU(ProcessingModule):
    
    def __init__ (self, name, mot_net, exp_name='' ):
        ProcessingModule.__init__(self, name)
        self.motivationNet = mot_net
        
        mu_number = 0
        groups=getGroupsFromNames()
        mu_names = []
        for group in groups:
            for neuron in group:
                mu_number +=1
                mu_names.append(neuron.name)
                #print(neuron.name)
        self.number_mu = mu_number
        
        import uuid
        self.file_name = "logs/reaCog_motivationNet_" + exp_name#str(uuid.uuid4())
        print("DATA COLLECTOR MU NAME: ", self.file_name)
        with open( self.file_name,'w') as f_handle:
            counter = 0 
            for item in mu_names:
                f_handle.write("%s" % item)
                counter += 1
                if (counter<self.number_mu):
                    f_handle.write("\t")
            f_handle.write('\n')
        
        #self.data_array = np.empty([self.data_points,23])#[None]*self.data_points
        
    ##
    #   Init Module
    def init_module(self):
        pass
    
    ##
    #   Read out data from motivation units 
    def post_processing_step(self, args=None):
        mu_activation_data = np.zeros(self.number_mu)
        mu_counter = 0
        groups=getGroupsFromNames()
        for group in groups:
            for neuron in group:
                mu_activation_data[mu_counter] = neuron.getActivation()
                #print(neuron.name)
                mu_counter += 1
        #input()
        with open( self.file_name,'ba') as f_handle:
            np.savetxt( f_handle, np.atleast_2d(mu_activation_data), fmt='%1.3f') 
            #np.savetxt( f_handle, test)
            #, fmt='%f %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')#, delimiter=',')