#== Ground Contact Parameters ========
#=====================================
""" In Walknet there are three different versions. Depending on the
	groundContactMethod flag it is decided which should be used:
	1. 	Predict current leg position (using fw kinematics) and
		simply decide if the leg should touch ground
		(very stable, but works only on flat terrain).
	2. 	Each leg is equipped (right now only in simulation) with a ground ground_contact
		sensor which provides the ground interaction force:
		positive value means the pressure which is acting on the sensor,
		when in the air the sensor just delivers a 0.
	3. Torsion based method
"""
from controller.reaCog.walknet.MotivationNetwork.GCSensorUnit import PredictedGroundContactSensorUnit, TarsusPressureGroundContactSensorUnit, TorsionBasedGroundContactSensorUnit
ground_contact_class_dict = {
    'PredictedGC': PredictedGroundContactSensorUnit,
    'TarsusPressureSensorGC': TarsusPressureGroundContactSensorUnit,
    'TorsionBasedGC': TorsionBasedGroundContactSensorUnit
}
groundContactMethod = 'PredictedGC'

# Parameters for torsion based method (number 3)
#groundcontact, if torque is above:
contacttorque1 = -9
#or (groundcontact if torque is above
contacttorque2 = -2
#and the foot in the z-axis is lower than
contactstancefact = 0.8
# times the stanceheight)
#after first contact, minimum time where the contact continues to exists
contactstancemin = 0.2
