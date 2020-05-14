import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np
import controller.reaCog.WalknetSettings as WSTATIC

##
#   Sub-Plot showing current leg position and current PEP position 
#   for every leg over time.
#
#   The visualization is a ProcessingModule which pulls in the post processing step
#   updated values from the bodyModel structure.
##
class PEPVisualizationPlayer():
    
    def __init__ (self, file_name):
        self.data_array = np.loadtxt(file_name) #, fmt='%f, %f, %f, %d, %d, %d, %d, %d, %d', delimiter=',')
        self.leg_order = [0,3,1,4,2,5]
        
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""
        if (self.data_array.shape[0] <= 1000):
            self.footfall = plt.figure(figsize=(12, 3))
        else:
            self.footfall = plt.figure(figsize=(16, 3))
        self.footfall.canvas.set_window_title('Footfall Pattern')
        self.ax_footfall = self.footfall.add_subplot(111)
        self.ax_footfall.set_yticks([6,5,4,3,2,1])
        self.ax_footfall.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size= 18)
        self.ax_footfall.set_xticks([200,400,600,800,1000,1200,1400,1600,1800])
        self.ax_footfall.set_xticklabels(['0.','2.', '4.','6.','8.', '10.','12.','14.','16.'], size=18)
  #          '24.','26.','28.','30.','32.'],size=18)
        #self.ax_footfall.set_xticklabels(['0.','2.', '4.','6.','8.', '10.', '12.','14.','16.'],size=18)
        
#        self.ax_footfall.set_xticks([200,600,1000,1400,1800,2200,2600,3000,3400,3800,4200,4600,5000,5400,5800,6200,6600])
 #       self.ax_footfall.set_xticklabels(['0.','4.', '8.','12.','16.', '20.','24.','28.','32.','36.','40.','44.',
  #          '48.','52.','56.','60.','64.'],size=18)
        
        #py.rcParams['figure.figsize'] = 4, 4
        self.ax_footfall.plot([0,self.data_array.shape[0]], [3.5,3.5], linestyle=':', linewidth=1.0, color='gray', alpha=0.7, marker='')
        self.ax_footfall.set_xlim(200, 1800)#self.data_array.shape[0]) 
        self.ax_footfall.set_ylim(0.5,6.5)

        draw_phase_box = False
        old_phase = 0
        initial_break = 50
        for it in range(0, self.data_array.shape[0]):
            current_phase = self.data_array[it][9]
            if (current_phase != old_phase):
                if (draw_phase_box):
                    if (current_phase == 1) and (old_phase==3):
                        self.ax_footfall.axvspan(phase_start, it, facecolor='r', alpha=0.4)
                    if (current_phase == 2):
                        self.ax_footfall.axvspan(phase_start-initial_break, it, facecolor='gray', alpha=0.4)
                        initial_break = 0
                    if (current_phase == 3):
                        self.ax_footfall.axvspan(phase_start, it, facecolor='gray', alpha=0.4)
                    if (current_phase == 4):
                        self.ax_footfall.axvspan(phase_start, it, facecolor='g', alpha=0.4)
                    if (current_phase == 0) and (old_phase == 4):
                        self.ax_footfall.axvspan(phase_start, it, facecolor='b', alpha=0.4)
                    if (current_phase == 0):
                        draw_phase_box = False
                else:
                    draw_phase_box = True
                #if not(2 <= current_phase <= 3):
                phase_start = it
            old_phase = current_phase

        for leg_nr in range(0,6):
            lastswing = -1
            for iter in range(0, self.data_array.shape[0]):
                if (self.data_array[iter][3+leg_nr] == 0) and lastswing<0 :
                    lastswing = iter
                elif (self.data_array[iter][3+leg_nr] == 1) and lastswing>=0:
                    self.ax_footfall.plot([lastswing,iter], [6-self.leg_order[leg_nr],6-self.leg_order[leg_nr]], linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")
                    lastswing = -1
        self.footfall.canvas.draw()
        self.footfall.savefig( file_name + "_footfall.pdf")
        print("Wrote footfall pattern " + str(file_name))
        plt.ioff()
        plt.close()
