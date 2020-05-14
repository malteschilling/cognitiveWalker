import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

##
#   Visualization of the Body Model.
#   Seen from above in a 2D view.
#   Only the legs which are touching the ground are drawn and shown.
#   The Visualization is realized as a ProcessingModule and pulling
#   itself all the necessary data from the bodyModel after this has been processed.
##
class BodyModel2DViewModule(ProcessingModule):
    
    def __init__ (self, name, bodyModel, motiv_net):
        ProcessingModule.__init__(self, name)
        self.bodyModel = bodyModel
        self.it = 0
        self.motivationNetRobot = motiv_net
        self.old_phase = -1
    
    ##
    #   Init of the figure (must be called before drawing method).
    def init_module(self):
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""
        self.fig_2d = plt.figure(figsize=(12, 6))
        self.ax_fig_2d = self.fig_2d.add_subplot(111)
        plt.axis('off')
        self.front_vect_line = [0,0,0,0,0,0]
        for i in range(0,6) :
#           self.ax_fig_2d.plot(self.bodyModel.get_leg_triangle(i)[0], self.bodyModel.get_leg_triangle(i)[1], linestyle=':', linewidth=1.0, color='gray', marker='')
            self.front_vect_line[i], = self.ax_fig_2d.plot(self.bodyModel.get_leg_triangle(i)[0], self.bodyModel.get_leg_triangle(i)[1], linewidth=1.0, color='gray', marker='o',alpha=0.7, mfc='gray')
#       py.xlim(-1.0,1.0)
#       py.ylim(-1.0,1.0)
        self.ax_fig_2d.set_xlim(3.0,-1.0) 
        self.ax_fig_2d.set_ylim(1.0,-1.0)
        plt.tick_params(
            axis='x',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            bottom='off',      # ticks along the bottom edge are off
            top='off',         # ticks along the top edge are off
            labelbottom='off')
        plt.tick_params(
            axis='y',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            left='off',      # ticks along the bottom edge are off
            right='off',         # ticks along the top edge are off
            labelleft='off')
        self.fig_2d.canvas.draw()
        plt.ioff()
#       plt.savefig("/Users/mschilling/Desktop/int_mod.pdf")
    
    ##
    #   Updating of the figure - use data from the body model and visualize this.
    def post_processing_step(self, args=None):
        for i in range(0,6) :
            #if (self.bodyModel.get_ground_contact(i)):
            if (self.bodyModel.get_leg_in_stance(i)):
                # Draw permanently current leg configuration
                #py.plot(self.get_leg_triangle(i)[0][0:3], self.get_leg_triangle(i)[1][0:3], linestyle='--', linewidth=1.0, color='gray', marker='')
                # Update current legs
                #self.front_vect_line[i].set_marker('o')
                py.setp(self.front_vect_line[i], linestyle='-', linewidth=2.0, color='black', marker='o',alpha=0.7, mfc='gray')
                self.front_vect_line[i].set_xdata(self.bodyModel.get_leg_triangle(i)[0][0:5])
                self.front_vect_line[i].set_ydata(self.bodyModel.get_leg_triangle(i)[1][0:5])  # update the data
            else:
                #self.front_vect_line[i].set_marker('None')
                py.setp(self.front_vect_line[i], linestyle=':', linewidth=2.0, color='black', marker='o',alpha=0.7, mfc='gray')
                self.front_vect_line[i].set_xdata(self.bodyModel.get_leg_triangle(i)[0][0:5])
                self.front_vect_line[i].set_ydata(self.bodyModel.get_leg_triangle(i)[1][0:5])
                #py.setp(self.front_vect_line[i], linestyle='', linewidth=1.0, color='gray', marker='',alpha=0.7, mfc='gray')
            #print(self.bodyModel.get_leg_triangle(i)[0][0:5], self.bodyModel.get_leg_triangle(i)[1][0:5])
        #print()
        self.fig_2d.canvas.draw()
        plt.pause(0.00001)
        current_phase = self.motivationNetRobot.cognitive_expansion.Phase.getCurrentPhase() 
#       if current_phase != self.old_phase:
#       if (840 < self.it < 860):
#           plt.savefig("/Users/mschilling/Desktop/int_mod_" + str(self.it) + ".pdf", transparent=True)
        self.old_phase = current_phase
        #plt.savefig("/Users/mschilling/Desktop/Figs/IM/im_posture_" + str(self.it) + ".pdf")
        self.it += 1
