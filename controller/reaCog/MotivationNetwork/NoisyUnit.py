from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
import random

##
#   NoisyUnit - used in the spreading activation layer.
#   The output_value is modulated my a multiplicative noise.
##
class NoisyUnit(MotivationUnit):

    random_factor = 0.2
    random_add = 1 - (random_factor/2) # was random_factor/2
        
    ##  Calculate current activation
    #   from the current inputs and the constant bias
    #   (automatically called in processing of network).
    def evaluate(self):
        self.output_value += self.input_sum
        if (self.output_value > 0):
            #self.output_value = (NoisyUnit.random_add + NoisyUnit.random_factor * random.random() ) * self.output_value
            # In order to force the SAL network towards one solution
            if (self.output_value > 1.):
                self.output_value = (NoisyUnit.random_add + NoisyUnit.random_factor * random.random() )
            # Maybe include some added noise to keep activity alive
            else:
                self.output_value = (NoisyUnit.random_add + NoisyUnit.random_factor * random.random() ) * self.output_value
        self.input_sum = 0