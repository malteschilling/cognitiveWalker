from .BodyModel.mmcBodyModel3D import mmcBodyModel
from .SwitchObject import SwitchObject
from .SwitchLeg import SwitchLeg

import controller.reaCog.WalknetSettings as WSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

##
# 	SwitchRobot
#
#	The Robot, WRobot structures handle sensory processing and sending of motor control 
#	signals. SwitchRobot wraps and hides these - it provides an interface to 
#	- either the robot (routing calls to robot and Wrobot)
#	- or the internal body model (when running decoupled in internal simulation)
#	This depends on the SwitchObject.
##  
class SwitchRobot (ProcessingModule):

    def __init__(self, wRobot):
        self.__dict__["forwarding"] = False
        self.wRobot = wRobot
        self.mmcRobot = mmcBodyModel(self.wRobot, WSTATIC.stability_threshold)
        
        ProcessingModule.__init__(self, "SwitchRobot")
        
        # Switch to original model
        # There is a boolean variable "decoupled" - initially False, but for 
        # internal simulation set to true
        self.switch = SwitchObject(self, False)
        # Create the switch legs - as some controller parts are using leg 
        # references, these has to be also channeled through a switch
        #self.frontLeftLeg = SwitchLeg(self.WRobot.frontLeftLeg, self.mmcRobot.frontLeftLeg, self.switch)
        
        temp_switchLegs=[]
        for wleg, leg_nr in zip(self.wRobot.wlegs, range(len(self.wRobot.wlegs))):
            temp_switchLeg = SwitchLeg(wleg, self.mmcRobot.mmcLegs[leg_nr], self.switch)
            setattr(self, wleg.leg.name, temp_switchLeg)
            temp_switchLegs.append(temp_switchLeg)
        self.wlegs=tuple(temp_switchLegs)
        
        self.motivationNetRobot = None
        
        self.__dict__["forwarding"] = True
        
    def __del__(self):
        pass

    ## Umlenken von Anfragen an die Instanz von Robot
    def __getattr__(self,name):
        #print("ROBOT switch: ", name)
        try :
            if self.switch.decoupled:
                return getattr(self.mmcRobot, name)
            else:
                return getattr(self.wRobot, name)
        except AttributeError :
            print("Konnte ",name, "nicht in WRobot oder BodyModel finden" )
            raise

    def __setattr__(self, name, value):
        if self.__dict__["forwarding"] is False:
            self.__dict__[name] = value
        elif name.startswith("__") and name.endswith("__"):
            self.__dict__[name] = value
        else:
            if name in self.__dict__:
                self.__dict__[name] = value
            else:
                if self.switch.decoupled:
                    setattr(self.mmcRobot, name, value)
                else:
                    setattr(self.wRobot, name, value)

    def set_motivation_net(self, motiv_net):
        self.motivationNetRobot = motiv_net
        self.mmcRobot.set_motivation_net(motiv_net)

    ##
    #   Initialization of the MMC leg models.
    def init_module(self):
        self.mmcRobot.init_leg_mmc_models()
        self.mmcRobot.updateLegStates()

    def pre_processing_step(self, timeStamp):
        self.motivationNetRobot.cognitive_expansion.Phase.getCurrentPhase()
        
    def processing_step(self, timeStamp):
        # The internal model for planning ahead should be only updated from the real body
        # when the switch is FALSE (not planning mode)
        if not(self.switch.decoupled):
            self.updateSensorStateToMMCLegs(1)

    def updateSensorStateToMMCLegs(self, steps):
        for mmc_leg in self.mmcRobot.mmcLegs:
            for i in range(0, steps):
                mmc_leg.add_sensory_joint_values([mmc_leg.wleg.leg.alpha.inputPosition, \
                    mmc_leg.wleg.leg.beta.inputPosition, mmc_leg.wleg.leg.gamma.inputPosition])

    def post_processing_step(self, timeStamp):
        # Iterate mmc legs
        if (self.switch.decoupled):
            for leg in self.mmcRobot.mmcLegs:
                for i in range(0, WSTATIC.mmc_mental_iterations):
                    leg.mmc_kinematic_iteration_step()
            # Iterate mmc body
            self.mmcRobot.updateLegStates() 
            for i in range(0,WSTATIC.mmc_mental_iterations):
                self.mmcRobot.mmc_iteration_step()
