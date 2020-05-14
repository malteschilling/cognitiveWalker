from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

##
#   ConnectionModulatorUnit are MotivationUnits and
#   do modulate connections between other Units.
#   Only when active (output_value > self.activation_threshold) the related
#   connections are active (transport signals).
#   Importantly, the modulated connections have to be registered at the modulating 
#   unit!
#
class ConnectionModulatorUnit(MotivationUnit):

    def __init__(self, name, startValue=0, bias=0, group_name=''):
        MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
        self.__modulatedConnectedTo = []
        self.activation_threshold = 0.5

    ##  Build modulated connection between two neurons, 
    #   needed is the source and the target neuron and a weight.
    def addModulatedConnectionBetweenNeurons(self, sourceNeuron, targetNeuron, weighting ):
        self.__modulatedConnectedTo.append((sourceNeuron, targetNeuron, weighting))

    def getModulatedConnections(self):
        return self.__modulatedConnectedTo

    ##  Propagate current activation to all connected neurons (weighted)
    #   during process execution.
    #   And when active, call the modulated connections
    def propagate(self):
        MotivationUnit.propagate(self)
        if (self.output_value > self.activation_threshold):
            for item in self.__modulatedConnectedTo:
                #print("Modulate: ", item[0].name, " / ", item[0].output_value)
                item[1].addIncomingValue( item[0].output_value * item[2] )