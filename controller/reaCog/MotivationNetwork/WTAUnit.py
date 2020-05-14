from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.ConnectionModulatorUnit import ConnectionModulatorUnit

##
#   WTA Network unit. Specific fast executing version.
#   In this version there are two passes:
#   - one, collecting the old overall activation
#   - the second one weighting the current activation of each unit 
#   As a consequence the complexity is only linear with increasing size of the network.
# #
class WTAUnitFast(MotivationUnit):

    wtaUnitList = []
    # Unit signaling when the network converged
    foundWinner = ConnectionModulatorUnit("WTAFast_converged")
    # Inhibition weight between participating units.
    weight_other = -1
    inhibition_sum = -1
    net_sum = 0
    
    def __init__(self, name, startValue=0, bias=0, group_name=''):
        MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
        WTAUnitFast.wtaUnitList.append(self)
        if (len(WTAUnitFast.wtaUnitList) > 1):
            WTAUnitFast.weight_other = -1. / (len(WTAUnitFast.wtaUnitList) - 1)

    ##  Propagate current activation to all connected neurons (weighted)
    #   during process execution.
    def propagate(self):
        MotivationUnit.propagate(self)
        # For the fast WTA:
        # Only once (when inhibition_sum not set = -1) all the activations in the WTA net are summed.
        # From this sum the inhibition can be calculated for each individual unit.
        if (WTAUnitFast.inhibition_sum == -1):
            WTAUnitFast.inhibition_sum = 0
            WTAUnitFast.net_sum = 0
            for i in range(0, len(WTAUnitFast.wtaUnitList) ):
                WTAUnitFast.inhibition_sum += WTAUnitFast.weight_other * WTAUnitFast.wtaUnitList[i].output_value
        self.input_sum += WTAUnitFast.inhibition_sum + (1 - WTAUnitFast.weight_other) * self.output_value
        if (self.input_sum > 0):
            # net_sum is used to keep track of all activations in the network for the next 
            # timestep - it is used for normalization in the applyOutput time step.
            WTAUnitFast.net_sum += self.input_sum
            
    def addIncomingValue(self, newValue):
        #self.__inputs.append(newValue)
        self.input_sum += newValue
        WTAUnitFast.net_sum += max(0., newValue)
            
    ##  Output function (automatically called in processing of network).
    #   Normalizes the summed input of all units to 1.
    def applyOutputFunction(self):
        WTAUnitFast.inhibition_sum = -1
        if (self.output_value > 0):
            if (WTAUnitFast.net_sum > 0):
                self.output_value = min( (self.output_value / WTAUnitFast.net_sum), 1.)
        else:
            self.output_value = 0.
        if (self.output_value > 0.9):
            WTAUnitFast.foundWinner.input_sum += 1.
        #list_act = ''
        #for wta_n in WTAUnitFast.wtaUnitList:
         #   list_act = list_act + str(wta_n.output_value) + " / "
        #print("WTA winner: ", WTAUnitFast.foundWinner.output_value, " / ", list_act)
        #        if (self.output_value > 0.95):
#            for otherUnits in WTAUnitFast.wtaUnitList:
 #               otherUnits.output_value = 0.
#            self.output_value = 1.
            
    def resetWTA(self):
        print("***** RESET THE WTA NETWORK ******")
        for wtaUnit in WTAUnitFast.wtaUnitList:
            wtaUnit.output_value = 0
            wtaUnit.input_sum = 0
        WTAUnitFast.net_sum = 0 
        WTAUnitFast.inhibition_sum = -1

##
#   Winner-Take-All Unit.
##
class WTAUnit(MotivationUnit):

    wtaUnitList = []
    outputCounter = 0
    outputSum = 1.
    foundWinner = ConnectionModulatorUnit("WTA_converged")
    
    def __init__(self, name, startValue=0, bias=0, group_name=''):
        MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
        WTAUnit.wtaUnitList.append(self)
        self.weight_vector = []
        if (len(WTAUnit.wtaUnitList) > 1):
            inhib_WTA_param = -1. / (len(WTAUnit.wtaUnitList) - 1)
        else: inhib_WTA_param = -1.
        for i in range(0, len(WTAUnit.wtaUnitList) ):
            WTAUnit.wtaUnitList[i].weight_vector = [inhib_WTA_param] * len(WTAUnit.wtaUnitList)
            WTAUnit.wtaUnitList[i].weight_vector[i] = 1.

    ##  Propagate current activation to all connected neurons (weighted)
    #   during process execution.
    def propagate(self):
        for i in range(0, len(self.weight_vector) ):
            self.input_sum += self.weight_vector[i] * WTAUnit.wtaUnitList[i].output_value
            
    ##  Output function (automatically called in processing of network).
    #   Normalizes the summed input of all units to 1.
    def applyOutputFunction(self):
        WTAUnit.outputCounter += 1
        if (WTAUnit.outputCounter >= len(self.weight_vector)):
            WTAUnit.outputSum = 0.
            WTAUnit.outputCounter = 0
            for unit in WTAUnit.wtaUnitList:
                if unit.output_value > 0:
                    WTAUnit.outputSum += unit.output_value
            for unit in WTAUnit.wtaUnitList:
                if (unit.output_value > 0):
                    if (WTAUnit.outputSum > 0):
                        unit.output_value = unit.output_value / WTAUnit.outputSum
                else:
                    unit.output_value = 0.
                if (unit.output_value > 0.9):
                    WTAUnit.foundWinner.input_sum = 1.
        list_act=''
        for wta_n in WTAUnit.wtaUnitList:
            list_act = list_act + str(wta_n.output_value) + " / "
        print("WTA winner: ", WTAUnit.foundWinner.output_value, " / ", list_act)

    def resetWTA(self):
        WTAUnit.outputCounter = 0
        WTAUnit.outputSum = 1
        for wtaUnit in WTAUnit.wtaUnitList:
            wtaUnit.output_value = 0
            wtaUnit.input_sum = 0
