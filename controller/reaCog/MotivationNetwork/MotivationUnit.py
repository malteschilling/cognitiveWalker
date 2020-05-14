#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Dictionary containing lists (groups) of neurons
# The keys are the names of the groups. The values are lists of neurons.
neuron_dict = dict()

##  Complete step of the neural network.
#   First propagate all the current activations in the network,
#   then evaluate the firing of the neurons and produce their
#   output value.
#   After that, the finalize-method is called for every neuron 
#   (executing the output function).
def executeNeuralNetStep(group_names=None):
    groups=getGroupsFromNames(group_names)
#   print("Execute Neural Net Step")
    for group in groups:
#       print("CONNECTIONS")
        for neuron in group:
            neuron.propagate()
#           for conn in neuron.getConnection():
#               if (conn[0].name=="swing_toBack_ml"):
#                   print(neuron.name, neuron.output_value, conn[1])
            #for conn in (neuron.getConnection()):
            #   print(conn[0].name, conn[1])

    for group in groups:
        for neuron in group:
            neuron.evaluate()
#           if (neuron.name=="swing_toBack_ml"):
#                   print("output_value ", neuron.output_value)

    for group in groups:
        for neuron in group:
            neuron.finalize()

## Add a neuron to the neuron dictionary
# If no group name is defined, an empty string will be used as name. 
def addNeuronToGroup(neuron, group_name=''):
    try:
        group=neuron_dict[group_name]
    except KeyError:
        group=[]
        neuron_dict[group_name]=group
    group.append(neuron)

## Get a list of neuron groups based on a list of group names.
# If only one group name shall be used, it can be used directly without the need to put it into a list first.
# If no group name is provided, the function returns a list of all groups.
def getGroupsFromNames(group_names=None):
    if group_names==None or ((isinstance (group_names, str) and group_names=='all')): # evaluate all Neurons
        groups=neuron_dict.values()
    else:
        if isinstance (group_names, str):
            group_names=[group_names]
        groups=[neuron_dict[group_name] for group_name in group_names]
    return groups

def getGroupNames():
    return neuron_dict.keys()

def getGroupNamesExceptFor(negative_group_names=None):
    if negative_group_names==None:
        return getGroupNames()
    if isinstance(negative_group_names,str):
        negative_group_names=set([negative_group_names])
    else:
        negative_group_names=set(negative_group_names)

    positive_group_names=[]
    for key in neuron_dict:
        if key not in negative_group_names:
            positive_group_names.append(key)
    return positive_group_names
    
## Runs only the propagate-step for all the specified group names.
# If no group names are provided, the function will execute the 
# propagate-method for every neuron in the neuron_dict.
def propagateNeuralNetStep(group_names=None):
    groups=getGroupsFromNames(group_names)
    for group in groups:
        for neuron in group:
            neuron.propagate()

## Runs only the evaluate-step for all the specified group names.
# If no group names are provided, the function will execute the 
# evaluate-method for every neuron in the neuron_dict.
def evaluateNeuralNetStep(group_names=None):
    groups=getGroupsFromNames(group_names)
    for group in groups:
        for neuron in group:
            neuron.evaluate()

## Runs only the finalize-step for all the specified group names.
# If no group names are provided, the function will execute the 
# finalize-method for every neuron in the neuron_dict.
def finalizeNeuralNetStep(group_names=None):
    groups=getGroupsFromNames(group_names)
    for group in groups:
        for neuron in group:
            neuron.finalize()


##  Run through the neural network and create a 
#   Graphivz Dot File which can be rendered as a nice figure.
def saveMotivationNetToDot():
    from controller.reaCog.MotivationNetwork.ConnectionModulatorUnit import ConnectionModulatorUnit
    fil = open("conn.dot","w")
    fil.write("digraph Conn {\nnodesep=0.7\n")
    for group in neuron_dict:
        for item in neuron_dict[group]:
            #print(item.name)
            for con in item.getConnection():
                link_attr = "["
                if con[1] < 0. :
                    link_attr += "arrowhead=\"dot\""
                both_directions=None
                for back_connection in con[0].getConnection():
                    if back_connection[0].name == item.name:
                        both_directions = back_connection
                if (both_directions==None):
                    if len(link_attr) > 1:
                        link_attr += "]"
                    else:
                        link_attr = ""
                    fil.write(str(item.name)+" -> "+str(con[0].name)+ link_attr + "\n")
                else:
                    #print("To : ", con[1], " - Back : ", con[0].name, " : ", both_directions[1])
                    if (item.name > con[0].name):
                        if len(link_attr)>1:
                            link_attr += ", "
                        if both_directions[1] < 0. :
                            link_attr += "arrowtail=\"dot\""
                        else:
                            link_attr += " arrowtail=\"normal\""
                        if len(link_attr) > 1:
                            link_attr += "]"
                        else:
                            link_attr = ""
                        fil.write(str(item.name)+" -> "+str(con[0].name)+ link_attr + "\n")
            # ModulatorUnits: These are also providing connections between units
            # but these are only active when the unit itself is active
            # Has to be drawn in addition as a dashed line.
            if (isinstance(item, ConnectionModulatorUnit)):
                for con in item.getModulatedConnections():
                    fil.write(str(con[0].name)+" -> "+str(con[1].name)+ "[style=dashed, label=\"" + item.name + "\", fontsize=10]\n")
    fil.write("\n}\n")
    fil.close()
    #import os
    #os.system("dot -Tpdf conn.dot -o \"conn.pdf\" ")

##
#   Neuron Object.
#   A generic neuron class which allows for connections between neurons.
#   Offers functions for building the structure of the network.
#   
#   A neuron is a summation unit which can have a static bias value and the 
#   output is the sum of all inputs plus bias.
##
class Neuron(object):
    ##
    #   Init
    #   @param startValue is the initial activation
    #   @param bias is the bias, i.e. the ongoing activation of the unit 
    def __init__(self, startValue=0, bias=0 , group_name=''):
        if group_name!=None:
            addNeuronToGroup(self, group_name)#neuronList.append(self)
        self.connectedTo =[]
        #self.__inputs = []
        self.input_sum = 0
        self.bias = bias
        #if startValue==0:
        #self.output_value = bias
        #else:
        self.output_value = startValue

    def __del__(self):
        pass

    ##  Build connection to another neuron, 
    #   needed is the target neuron and a weight.
    #   @param connectedNeuron target neuron of the connection
    #   @param weighting connection strength
    def addConnectionTo(self, connectedNeuron, weighting ):
        self.connectedTo.append((connectedNeuron, weighting))

    #   Remove connection to another neuron.
    def removeConnectionTo(self, rmListener):
        for item in self.connectedTo:
            if rmListener in item:
                self.connectedTo.remove(item)
                
    ##  Change connection weight to another neuron.
    def changeConnectionWeight(self, rmListener, newWeight):
        notFound = True
        for item in range(0, len(self.connectedTo)):
            if rmListener in self.connectedTo[item]:
                self.connectedTo[item]=(rmListener, newWeight)
                notFound = False
        if (notFound):
            print("When trying to change connection weight - did not find connection to : ", rmListener.name)
                
    def getConnection(self):
        return self.connectedTo

    ##  Propagate current activation to all connected neurons (weighted)
    #   during process execution.
    def propagate(self):
        if self.output_value != 0 :
            for neuron, weight in self.connectedTo:
                try:
                    neuron.addIncomingValue( self.output_value * weight )
                except AttributeError as err1:
                    try:
                        neuron(self.output_value * weight)
                    except Exception as err2:
                        print(err1)
                        raise err2
    
    ##  Adds inputs.
    #   During the propagate phase of the network process the inputs are accumulated
    #   i.e. they are simply summed (in the past they were collected and later summed
    #   which is not so efficient, but might be necessary if non linear integration
    #   should be applied)  
    #   @param incoming activation (which shall be added)
    def addIncomingValue(self, newValue):
        #self.__inputs.append(newValue)
#       if (self.name == "swing_toFront_hr"):
#           print("******** SWING TO FRONT ADD = ", self.input_sum, newValue)
        self.input_sum += newValue
        
    ##  Calculate current activation
    #   from the current inputs and the constant bias
    #   (automatically called in processing of network).
    def evaluate(self):
        self.output_value = self.bias + self.input_sum

    ##  Execute the applyOutputFunction.
    def finalize(self):
        self.applyOutputFunction()
        self.input_sum = 0
        
    ##  Output function (automatically called in processing of network).
    def applyOutputFunction(self):
        # As a standard this is a linear unit.
        # A derived class just overwrites this function.
        #self.output_value = self.output_value
        pass

    def getActivation(self):
        return self.output_value
    
    def setActivation(self, output_value):
        self.output_value=output_value

##
#   Motivation Unit
#   is a Neuron also usually (in this generic case)
#   summing up the incoming values, but restricting the output
#   to the range of [0,1].
#   
#   In an extension motivation units work as a modulating unit switching on
#   and off behaviours (ModulatingMotivationUnit).
##
class MotivationUnit(Neuron):
    def __init__(self, name, startValue=0, bias=0, group_name=''):
        Neuron.__init__(self, startValue=startValue, bias=bias, group_name=group_name)
        self.name = name
    
    ##  Calculate current activation
    #   from the current inputs and the constant bias
    #   (automatically called in processing of network).
#   def evaluate(self):
#       if (self.name == "pD_MMC_unstableHL_delay"):
#           print("DELAY EVALUATE = ", self.input_sum, self.bias, self.output_value)
#       self.output_value = self.bias + self.input_sum
    
    ##  Propagate current activation to all connected neurons (weighted)
    #   during process execution.
    def propagate(self):
        if self.output_value != 0 :
            for neuron, weight in self.connectedTo:
                #if neuron.name=="swing_toBack_mr":
                #    print("  -> Add input to toBack: ", self.name, self.output_value, weight )
                #if neuron.name=="swing_toFront_mr":
                #    print("  -> Add input to toFront: ", self.name, self.output_value, weight )
                try:
                    neuron.addIncomingValue( self.output_value * weight )
                except AttributeError as err1:
                    try:
                        neuron(self.output_value * weight)
                    except Exception as err2:
                        print(err1)
                        raise err2
    
    ##  Output function (automatically called in processing of network).
    #   Restricts value range to 0<=output<=1.
    def applyOutputFunction(self):
#       if self.name=="problem_detector":
#                   print("  Output problem_detector: ", self.output_value)
        # As a standard this is a linear unit.
        # A derived class just overwrites this function.
        if self.output_value >1:
            self.output_value = 1
        elif self.output_value<0 :
            self.output_value = 0
            
class ModulatedParameterProviderUnit(MotivationUnit):
    def __init__(self, name, startValue=0, bias=0, group_name=''):
        MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
        self.modulated_external_param_provider = None
        
    def set_modulated_external_param_provider(self, param_provider):
        self.modulated_external_param_provider = param_provider

    def get_modulated_external_param(self, param_name):
        return (self.output_value * getattr(self.modulated_external_param_provider, param_name) )
