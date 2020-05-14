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

# assume gc after the first touchdown until swinging is initialized:
#assume_gc_during_stance_phase = True
# assume no gc during the swing as long as the beta joint lifts the leg:
#assume_no_gc_while_lifting_leg = True

# Parameter for prediction based method
# parameter describes predicted height value from which on 
# a leg is predicted as in gc - is given as a percentage
# (good value: 0.8, means when leg is having a height of 0.8 * the 
# intended height control value it is already assumed as having ground contact
# predicted_ground_contact_height_factor is defined in WalknetSettings

# Parameter for tarsus pressure sensor
# ground contact (gC) detection by force threshold:
# tarsusPressureGCThreshold is defined in WalknetSettings

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
