from controller.reaCog.MotivationNetwork.NoisyUnit import NoisyUnit
from controller.reaCog.MotivationNetwork.WTAUnit import WTAUnitFast
from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.ConnectionModulatorUnit import ConnectionModulatorUnit
from controller.reaCog.MotivationNetwork.PhasicUnit import PhasicUnit

from .CognitivePhases import CognitivePhases

##
# 	SpreadAndSelectNetwork
#
#	Defines the three layers of the cognitive expansion.
#	For each MotivationUnit connected to a behavior there is a corresponding unit
#	in these three layers:
#	- SAL = spreading of activation layer, search for alternative behaviors
#		that are close by to the behavior causing the problem
#	- WTA = selection of a single behavior activated in SAL (using a Winner-Take-All
#		structure)
#	- RTB = remember behaviors that have been tested in internal simulation 
#		which inhibits selecting these behaviors again in the WTA layer.
#
#	The SAL and WTA layers are connected to specific phases during planning ahead that
# 	are realized in CognitivePhases.
#
#	For details on the controller structure see submitted publication 
#  	or for an older version: Schilling and Cruse, 2017 "ReaCog, a Minimal 
#	Cognitive Controller Based on Recruitment of Reactive Systems" in Frontiers in 
#	Neurorobotics.
##  
class SpreadAndSelectNetwork(object):

    def __init__(self, orig_MUs, cog_phases):
        self.Phase = cog_phases
        self.original_MUs = orig_MUs
        
        WTAUnitFast.foundWinner.addConnectionTo(self.Phase.MU_Sim, 1)
        WTAUnitFast.foundWinner.addConnectionTo(self.Phase.MU_WTA, -2)
        self.Phase.MU_SAL.addConnectionTo(WTAUnitFast.foundWinner, -1)
        
        # Modulating the input to the idea units after a solution is found.
        # This is done only for a short time window
        self.modulate_idea_input = ConnectionModulatorUnit("modulate_idea_input", group_name='cognitive_layer')
        self.modulate_idea_input_time_window = PhasicUnit("modulate_idea_input_time_window", group_name='cognitive_layer', time_window =20)
        self.Phase.MU_Sim.addConnectionTo(self.modulate_idea_input_time_window, 1)
        self.Phase.MU_TestBeh.addConnectionTo(self.modulate_idea_input_time_window, 1)
        self.Phase.MU_SAL.addConnectionTo(self.modulate_idea_input, -1)
        self.modulate_idea_input_time_window.addConnectionTo(self.modulate_idea_input, 1)       
        
        self.sal_weight = 0.3
        self.layer_SAL = []
        self.layer_WTA = []
        # Remember Tested Behavior
        self.layer_RTB = []
        for i in range(len(self.original_MUs)):
            #   Spreading Activation Layer:
            #   1. Units are activating neighboring units - this activation
            #       is modulated by a spreading_modulator Motivation unit
            #       (only when this is active the activation is spreading inside the layer)
            #   2. There is a recurrent connection onto itself (weight = 1)
            #       meaning the units are integrating inputs.
            #   3. Units have a random component (noise)
            self.layer_SAL.append( NoisyUnit( ("SAL_"+ str(self.original_MUs[i].name[0:2] 
                + self.original_MUs[i].name[self.original_MUs[i].name.index("_")+3] 
                + self.original_MUs[i].name[-3:]) ), 
                group_name='cognitive_expansion') )
    
            if (i>0):
                self.Phase.MU_SAL.addModulatedConnectionBetweenNeurons( self.layer_SAL[-1], self.layer_SAL[-2], self.sal_weight )
                self.Phase.MU_SAL.addModulatedConnectionBetweenNeurons( self.layer_SAL[-2], self.layer_SAL[-1], self.sal_weight )
        
            self.layer_WTA.append( WTAUnitFast( ("WTA_"+ str(self.original_MUs[i].name[0:2] 
                + self.original_MUs[i].name[self.original_MUs[i].name.index("_")+3] 
                + self.original_MUs[i].name[-3:]) ), group_name='cognitive_expansion') )
            # The already active MotivationUnits (original, part of behavior selection) 
            # shall not be selected and therefore inhibit the selection on the WTA layer
            # Removed: first, testing a single behavior might 
            # still be sensible.
            # second, the WTA unit gets too much inhibition during simulation
            # and would not be active at beginning of test of behavior
            # self.original_MUs[i].addConnectionTo(self.layer_WTA[-1], -0.2)
            # The selected winner gets activated
            self.modulate_idea_input.addModulatedConnectionBetweenNeurons(self.layer_WTA[i], self.original_MUs[i], 2.)
            
            self.Phase.MU_SAL.addModulatedConnectionBetweenNeurons( self.layer_SAL[-1], self.layer_WTA[-1], 0.5 ) #was 0.5
    
            self.layer_RTB.append( MotivationUnit( ("RTB_"+ str(self.original_MUs[i].name[0:2] 
                + self.original_MUs[i].name[self.original_MUs[i].name.index("_")+3] 
                + self.original_MUs[i].name[-3:]) ) ) )
            WTAUnitFast.foundWinner.addModulatedConnectionBetweenNeurons( self.layer_WTA[-1], self.layer_RTB[-1], 1 )
            self.layer_RTB[-1].addConnectionTo(self.layer_WTA[-1], -0.2)
            self.layer_RTB[-1].addConnectionTo(self.layer_RTB[-1], 1)
	
	# Reset function before performing a new search for an alternative behavior
    def resetWTA(self):
        self.layer_WTA[0].resetWTA()
    
    # Reset function, called when internal simulation ended
    def resetSAL(self):
        for salUnit in self.layer_SAL:
            salUnit.output_value = 0
            salUnit.input_sum = 0
            
    # Reset function, called when internal simulation ended
    def resetRTB(self):
        for rtbUnit in self.layer_RTB:
            rtbUnit.output_value = 0
            rtbUnit.input_sum = 0

	# For future extension when a winner shall be learnt.
    def learnCurrentWinner(self, activation):
        winner = -1
        for i in range(len(self.layer_WTA)):
            if self.layer_WTA[i].output_value == 1. :
                winner = i
        if winner >= 0:
            print("Found solution and learn activation: ", winner)
            self.original_MUs[winner].addConnectionTo(self.original_MUs[winner], 1)
            self.original_MUs[winner].addIncomingValue(1)
