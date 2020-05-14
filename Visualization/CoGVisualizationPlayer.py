import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

##
#   Sub-Plot showing current leg position and current PEP position 
#   for every leg over time.
#
#   The visualization is a ProcessingModule which pulls in the post processing step
#   updated values from the bodyModel structure.
##
class CoGVisualizationPlayer():
    
    def __init__ (self, file_name):
        self.data_array = np.loadtxt(file_name) #, fmt='%f, %f, %f, %d, %d, %d, %d, %d, %d', delimiter=',')
        
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""

        self.stab_fact = plt.figure(figsize=(10, 3))
        self.stab_fact.canvas.set_window_title('Stability factor')
        self.ax_stab = self.stab_fact.add_subplot(111)
        #self.ax_stab.plot([0,1000], [3.5,3.5], linestyle=':', linewidth=1.0, color='gray', alpha=0.7, marker='')
        self.stab_line, = self.ax_stab.plot([],[], color='blue')
        self.time_window = self.data_array.shape[0]
        self.ax_stab.set_xlim(0, self.time_window) 
        self.ax_stab.set_ylim(-2.,2.)

        pos = [0]*self.time_window
        max_stab = -1.
        for i in range(1, self.time_window):
            pos[i] = self.data_array[i][16]
            if (pos[i] > max_stab):
                max_stab = pos[i]
        pos[0] = pos[1]
        position_line, = self.ax_stab.plot(range(self.time_window), pos, color='blue')
        self.ax_stab.text(10, 1.5, max_stab)

        self.stab_fact.canvas.draw()
        self.stab_fact.savefig(str(file_name) + "_stab_fact.pdf")
        print("Wrote CoG graph " + str(file_name))
        plt.ioff()
        plt.close()