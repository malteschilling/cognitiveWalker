import comminter
import sys
from experiment_main import init_environment

import time

##
# EXPERIMENT: Undisturbed walking.
#
# Walking velocity is set in the WalknetSettings.py (in controller/reaCog)
# - parameter default_speed. This sets the speed of the stance movement as mediated through
# the internal model.
#
# The experiment calls the main method for setting up the environment, the robot and the 
# controller. It further initializes experimental settings. In undisturbed walking no 
# disturbance is applied to the simulation. The robot starts in a default starting posture.
# Importantly, this is not optimal for most speeds and a coordination pattern has to 
# emerge over time.
#
# One can further specify a specific startPostureNumber (from the experiments that
# systematically check for all possible combinations of start postures).
##

##  Getting the command line arguments. 
def _args():
    import argparse
    parser = argparse.ArgumentParser(description="WALKNET start skript ")
    parser.add_argument("-t", "--simulationDuration", action="store", default=20., type=float,
        dest="simulationDuration", help="Duration of the simulation in seconds (in simulation time).")
    parser.add_argument("-l", "--log", action="store_true", default=False,
        dest="log", help="Log output.")
    parser.add_argument("-s", "--Startposture", action="store", default=0, type=int,
        dest="startPostureNumber", help="Configuration of initial posture (given for each leg as number from 0 to 4)")
    return parser.parse_args()

## Main method
if __name__ == "__main__":
    if sys.version_info < (3, 2):
        print("This programm requests Python 3.2 or higher")
        quit()
    args = _args()

    start_posture = args.startPostureNumber
    #start_posture = 1687
    
    simulation_duration = args.simulationDuration
    
    # Create the communication interface
    communication_interface=comminter.CommunicationInterface()
    protocolXmlDirectory="../hector/BioFlexBusProtocolXmls/"
    communication_interface.ParseProtocolXmls([protocolXmlDirectory+protocolXml for protocolXml in ["BIOFLEX_1_PROT.xml", "BIOFLEX_ROTATORY_1_PROT.xml", "BIOFLEX_ROTATORY_CONTROL_1_PROT.xml", "BIOFLEX_ROTATORY_ERROR_PROT.xml", "SIMSERV_1_PROT.xml", "IMU_PROT.xml", "PRESSURE_SENSOR_PROT.xml"]]) # Parse the xml files that contain the command-protocol definitions
    
    # Introducing a DISTURBANCE: passed as a list to the environment
    # First element = after what point in time should the disturbance be introduced?
    # Second element = a list of which legs should be affected.
    # For details see experiment_main.py
    #randomLongStep = [5.4, [2]] # exp #my_list[0] Exp 5.84 3 4 = 894 Exp 3.53 3 4 = 
    randomLongStep = []
    if (randomLongStep):
        randomLongStep_name = str(randomLongStep[0]) + " " + " ".join(str(i) for i in randomLongStep[1])
    else:
        randomLongStep_name = "None"
    #for posture_number in range(0, start_posture_list.shape[0]):
    print("*************************************")
    print("Start experiment : " + str(start_posture))
    print("*************************************")
    init_environment(communication_interface, start_posture, randomLongStep, simulation_duration)
    
    # After finishing the single experiment, create the visualizations.
    # Plot footfall patterns (and highlight planning stages as shaded areas).
    from Visualization.PEPVisualizationPlayer_allPhases import PEPVisualizationPlayer
    pepVisualization = PEPVisualizationPlayer( "logs/reacog_walking_" + "posture_var_" + str(start_posture).zfill(4) )

    # Plot leg position from top view.
    from Visualization.PositionVisualizationPlayer import PositionVisualizationPlayer
    posVisualization = PositionVisualizationPlayer( "logs/reacog_walking_" + ("posture_var_" + str(start_posture).zfill(4)))
    # Plot Center of gravity along body axis over time.
    from Visualization.CoGVisualizationPlayer import CoGVisualizationPlayer
    cogVisualization = CoGVisualizationPlayer( "logs/reacog_walking_" + ("posture_var_" + str(start_posture).zfill(4)))