from .BodyModel.mmcBodyModel3D import mmcBodyModel
from .SwitchObject import SwitchObject
from .SwitchLeg import SwitchLeg

import controller.reaCog.WalknetSettings as WSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

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

#   def sendAllAngleVelocity(self):
#       #print("SWITCH ANGLE VEL")
#       #for leg in self.mmcRobot.mmcLegs:
#       for leg, leg_nr in zip(self.mmcRobot.mmcLegs, range(len(self.mmcRobot.mmcLegs))):
#           leg.addControlVelocities(self.wRobot.wlegs[leg_nr].controlVelocities )
#           leg.sendControlVelocities()
#       self.mmcRobot.sendAllAngleVelocity()
#       self.wRobot.sendAllAngleVelocity()

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
        #print("LEG:     ", self.mmcRobot.front_left_leg.wleg.leg.alpha.inputPosition, self.mmcRobot.front_left_leg.wleg.leg.beta.inputPosition, self.mmcRobot.front_left_leg.wleg.leg.gamma.inputPosition)
        #print("MMC LEG: ", self.mmcRobot.front_left_leg.getInputPosition())
        # The internal model for planning ahead should be only updated from the real body
        # when the switch is FALSE (not planning mode)
        if not(self.switch.decoupled):
            self.updateSensorStateToMMCLegs(1)
            #print("MMC LEG: ", self.mmcRobot.front_left_leg.getInputPosition())

    def updateSensorStateToMMCLegs(self, steps):
        for mmc_leg in self.mmcRobot.mmcLegs:
            for i in range(0, steps):
                mmc_leg.add_sensory_joint_values([mmc_leg.wleg.leg.alpha.inputPosition, \
                    mmc_leg.wleg.leg.beta.inputPosition, mmc_leg.wleg.leg.gamma.inputPosition])
        #       mmc_leg.mmc_kinematic_iteration_step()
        #self.mmcRobot.updateLegStates()    

    def post_processing_step(self, timeStamp):
#2        if (self.mmcRobot.debug):
#2            print("POSTPROCESS SWITCH ROBOT STEP")
#2            for mleg in self.motivationNetRobot.motivationNetLegs:
#2                print("GC: ", mleg.wleg.mmcLeg.predictedGroundContact(), mleg.wleg.mmcLeg.input_foot_position )
#2            print("ML: ", self.motivationNetRobot.motivationNetLegs[2].swing_net.iteration, self.motivationNetRobot.motivationNetLegs[2].swing_net.saved_iteration)
#2            input()
    
        #self.mmcRobot.mmc_iteration_step()
        # Iterate mmc legs
        if (self.switch.decoupled):
            for leg in self.mmcRobot.mmcLegs:
                for i in range(0, WSTATIC.mmc_mental_iterations):
                    leg.mmc_kinematic_iteration_step()
#       print("MMC LEG: ", self.mmcRobot.front_left_leg.getInputPosition(), self.mmcRobot.front_left_leg.getFootPosition() )
#       print("WLEG   : ", self.wRobot.front_left_leg.leg.getInputPosition(), self.wRobot.front_left_leg.leg.input_foot_position)
            # Iterate mmc body
            self.mmcRobot.updateLegStates() 
            for i in range(0,WSTATIC.mmc_mental_iterations):
                self.mmcRobot.mmc_iteration_step()
        #self.bodyModelStance.check_static_stability_along_segment()
        #for motiv_leg in self.motivationNetLegs:
        #   if (motiv_leg.wleg.leg.leg_enabled):
        #       motiv_leg.update_leg_mmc_model()
        
#       self.front_left_leg.add_sensory_joint_values([self.robot.front_left_leg.alpha.getTargetAngle(),
#           self.robot.front_left_leg.beta.getTargetAngle(), self.robot.front_left_leg.gamma.getTargetAngle()])
#       self.front_left_leg.mmc_kinematic_iteration_step()
        #HERE THE VELOCITIES SHOULD BE APPLIED = setAngleVelocity has been called on the leg
        # and through the switch piped to the mmcLeg - from there now 
        # applied as a displacement = velocity * 0.01
        
        #WLeg(robot.front_left_leg, stability,self.__bodyModel, 0, 2)
        #self.front_right_leg = mmcAngularLegModel(robot.front_right_leg)
        #self.middle_left_leg = mmcAngularLegModel(robot.middle_left_leg)
        #self.middle_right_leg = mmcAngularLegModel(robot.middle_right_leg)
        #self.hind_left_leg  = mmcAngularLegModel(robot.hind_left_leg)
        #self.hind_right_leg  = mmcAngularLegModel(robot.hind_right_leg)
#       self.mmcRobot.mmc_iteration_step()
#        if (self.mmcRobot.debug):
 #           for leg in self.mmcRobot.mmcLegs:
  #              print(leg.controlVelocities, leg.input_foot_position[2])
   #         for mleg in self.motivationNetRobot.motivationNetLegs:
    #            print("GC: ", mleg.wleg.realLeg.predictedGroundContact() )
     #           print("SWING: ", mleg.swing_motivation_toFront.output_value )
      #          print("STANCE: ", mleg.stance_motivation_toBack.output_value )
       #     input()

#   def __setattr__(self,name, value):
#       if self.__dict__["forwarding"] == False:
#           self.__dict__[name] = value
#       else :
#           if name in self.__dict__ :
#               self.__dict__[name] = value
#           else :
#               if self.switch==0:
#                   setattr(self.mmcLeg, name, value)
#               else:
#                   setattr(self.WLeg, name, value)
