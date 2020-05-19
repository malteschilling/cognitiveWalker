from controller.reaCog.walknet.MotivationNetwork.SensorUnit import SensorUnit

##
#	A CollisionDetector unit is a Sensor Unit.
#	The activation solely depends on sensor readings -- here, when the leg is in swing
#	it is at the same time sensitive to hitting obstacles and experiencing unexpected 
# 	forces. These would trigger the unit.
#	The sensor value is immediately calculated before its propagation.
##  
class CollisionDetector(SensorUnit):
	def __init__(self, motivLeg, group_name=''):
		SensorUnit.__init__(self, name="collisionDetector_" + motivLeg.wleg.leg.name[1:3], group_name=group_name)
		self.motivationNetLeg = motivLeg
	
	# Checking for collision and trigger the unit	
	def updateSensorValue(self):
		if self.motivationNetLeg.wleg.leg.leg_enabled and self.motivationNetLeg.isSwinging():
			force=self.motivationNetLeg.wleg.leg.estimateForceFromTorsion()
			if force[0]>10:
				self.output_value =1
			else:
				self.output_value =0
		
		
		
