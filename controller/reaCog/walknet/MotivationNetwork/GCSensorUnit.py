from controller.reaCog.walknet.MotivationNetwork.SensorUnit import SensorUnit
from ... import WalknetSettings as WSTATIC
""" A GroundContact Sensor unit.
    
    For a given leg, the unit represents the ground contact signal in the 
    neural network.
    
    In Walknet there are three different versions. Depending on the
    groundContactMethod flag it is decided which should be used:
    1.  Predict current leg position (using fw kinematics) and
        simply decide if the leg should touch ground
        (very stable, but works only on flat terrain).
    2.  Each leg is equipped (right now only in simulation) with a ground ground_contact
        sensor which provides the ground interaction force:
        positive value means the pressure which is acting on the sensor,
        when in the air the sensor just delivers a 0.
    3. Torsion based method
""" 
class GroundContactSensorUnit(SensorUnit):

    def __init__(self, motivLeg, group_name=''):
        SensorUnit.__init__(self, name="gc_" + motivLeg.wleg.leg.name[0] + motivLeg.wleg.leg.name[motivLeg.wleg.leg.name.index('_')+1], group_name=group_name)
        self.motivationNetLeg = motivLeg

""" Predicted Ground Contact
    Predict current leg position (using fw kinematics) and
    simply decide if the leg should touch ground
    (very stable, but works only on flat terrain).
    
    Calls the walknet predictedGroundContact function
    (parameter for this function are therefore in WalknetSettings)
"""     
class PredictedGroundContactSensorUnit(GroundContactSensorUnit):
    
    ##  Pull ground contact value from the connected leg.
    def updateSensorValue(self):
        #print("Pred GC: ", self.name, self.motivationNetLeg.wleg.predictedGroundContact() )
        if WSTATIC.searching_movements:
            if (self.output_value > 0.):
                self.output_value = self.motivationNetLeg.wleg.predictedGroundContact()
            else:
                # Maybe look for only end of swing movement
                #if (self.motivationNetLeg.wleg.leg.input_foot_position[0] > ):
                # Check if there is a force measured supporting the body
                if (self.motivationNetLeg.wleg.leg.estimateForceFromTorsion()[2] < 0.):
                    self.output_value = self.motivationNetLeg.wleg.predictedGroundContact()
                #if ("front_left" in self.motivationNetLeg.wleg.leg.name):
                 #   print(self.output_value, self.motivationNetLeg.wleg.leg.input_foot_position[0], 
                  #      " force: ", self.motivationNetLeg.wleg.leg.estimateForceFromTorsion(),
                   #     " torque: ", self.motivationNetLeg.wleg.leg.joints[1].torsion)
        else:
            self.output_value = self.motivationNetLeg.wleg.predictedGroundContact()

""" Use PressureSensor at Tarsus to determine ground contact.
    Each leg is equipped (right now only in simulation) with a ground ground_contact
    sensor which provides the ground interaction force:
    positive value means the pressure which is acting on the sensor,
    when in the air the sensor just delivers a 0.
    
    Calls the walknet tarsusPressureSensorGroundContact function
    (parameter for this function are therefore in WalknetSettings)
"""
class TarsusPressureGroundContactSensorUnit(GroundContactSensorUnit):
    
    ##  Pull ground contact value from the connected leg.
    def updateSensorValue(self):
        self.output_value = self.motivationNetLeg.wleg.tarsusPressureSensorGroundContact()
#       if (self.output_value == 0):
#           self.motivationNetLeg.motivationNetRobot.bodyModelStance.lift_leg_from_ground(self.motivationNetLeg.wleg.leg_nr)
        #print("GC: ", self.output_value, self.motivationNetLeg.stance_motivation.output_value , self.motivationNetLeg.wleg.leg.tarsus_pressure_sensor.highestPressure)



""" Torsion Based estimation of ground contact
"""
class TorsionBasedGroundContactSensorUnit(GroundContactSensorUnit):
    def __init__(self, motivLeg, group_name=''):
        GroundContactSensorUnit.__init__(self, motivLeg=motivLeg, group_name=group_name)
    ##  Pull ground contact value from the connected leg.
    def updateSensorValue(self):
        if not self.motivationNetLeg.wleg.leg.leg_enabled:
            self.output_value = 0
            return
        
        force=self.motivationNetLeg.wleg.leg.estimateForceFromTorsion()
        if force[2]<-10:
            self.output_value = 1
        else:
            self.output_value = 0
        #print("GC: ", self.output_value, self.motivationNetLeg.stance_motivation.output_value , self.motivationNetLeg.wleg.leg.tarsus_pressure_sensor.highestPressure)

