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
class PositionVisualizationPlayer():
    
    def __init__ (self, file_name):
        self.data_array = np.loadtxt(file_name) #, fmt='%f, %f, %f, %d, %d, %d, %d, %d, %d', delimiter=',')
        self.leg_order = [0,3,1,4,2,5]
        
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""
        plt.ion()
        self.fig_position = plt.figure(figsize=(15, 16))
        self.position_visualization = [0,0,0,0,0,0]
        self.time_window = self.data_array.shape[0]
        
        plot_nr = 321
        aep = WSTATIC.front_initial_aep[0]
        pep = WSTATIC.front_initial_pep[0]
        pos = [0]*self.time_window
        for leg_nr in range(0,6):
            ax_fig = self.fig_position.add_subplot(plot_nr)
            
            for i in range(0, self.time_window):
                pos[i] = self.data_array[i][17+leg_nr]
            #ax_fig.plot(range(self.time_window), pos, color='blue')
            ax_fig.plot([0,self.time_window],[pep, pep], linestyle=':', linewidth=1.0, color='blue')
            ax_fig.plot([0,self.time_window],[aep, aep], linestyle=':', linewidth=1.0, color='blue')
            
            draw_phase_box = False
            old_phase = 0
            for it in range(0, self.time_window):
                current_phase = self.data_array[it][9]
                if (current_phase != old_phase):
                    if (draw_phase_box):
                        if (current_phase == 1) and (old_phase==3):
                            ax_fig.axvspan(phase_start, it, facecolor='r', alpha=0.4)
                        if (current_phase == 4):
                            ax_fig.axvspan(phase_start, it, facecolor='g', alpha=0.4)
                        if (current_phase == 0) and (old_phase == 4):
                            ax_fig.axvspan(phase_start, it, facecolor='b', alpha=0.4)
                        if (current_phase == 0):
                            draw_phase_box = False
                    else:
                        draw_phase_box = True
                    if not(2 <= current_phase <= 3):
                        phase_start = it
                old_phase = current_phase
            
            for i in range(0, self.time_window):
                pos[i] = self.data_array[i][10+leg_nr]
            position_line, = ax_fig.plot(range(self.time_window), pos, color='green')
            
            ax_fig.set_xlim(0,self.time_window) 
            ax_fig.set_ylim( (pep-0.1), (aep+0.05) )
            plt.locator_params(nbins=8)
            
            plot_nr += 1
            if (plot_nr == 323):
                aep = WSTATIC.middle_initial_aep[0]
                pep = WSTATIC.middle_initial_pep[0]
            if (plot_nr == 325):
                aep = WSTATIC.hind_initial_aep[0]
                pep = WSTATIC.hind_initial_pep[0]
                    
        plt.tight_layout(h_pad=6, w_pad=4)
        #plt.ioff()
        #self.fig_position.canvas.draw()
        self.fig_position.savefig(str(file_name) + "_pos_vis.pdf")
        #plt.ioff()
        #plt.close()
