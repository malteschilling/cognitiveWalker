from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

class DelayCircuit():

    def __init__(self, name, input_unit, delay=5, group_name=''):
        self.delay = 1.*delay
        self.input_unit = input_unit
        
        # First unit: is increasing activity over time (recurrent connection)
        self.circ_count = MotivationUnit(name="circ_counter_"+name, bias=-(self.delay/(self.delay+1)), group_name=group_name)
        self.circ_count.addConnectionTo(self.circ_count, 1.)
        self.input_unit.addConnectionTo(self.circ_count, 1.)
        
        # Output unit: delayed signal
        self.circ_delayed = MotivationUnit(name="circ_delayed_"+name, bias=-delay, group_name=group_name)
        self.circ_count.addConnectionTo(self.circ_delayed, (delay+1))