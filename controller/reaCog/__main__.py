import comminter
import sys
import inspect, os
import numpy
from controller.reaCog.walknet.walknetF import Walknet
from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import SimulatorTimerModule as SimulatorTimerModule
from Hector.RobotF import Robot
from controller.reaCog.walknet.WRobotF import WRobot
#from ExternalControlInterface import ExternalControlInterface
#from DataServer import DataServer

from Hector import RobotSettings as RSTATIC
from controller.reaCog import WalknetSettings as WSTATIC
from controller.PartialMovements.WaitForInput import WaitForInput
from controller.PartialMovements.AdjustHeightOfRobot import AdjustHeightOfRobot
from controller.PartialMovements.MoveSingleLeg import MoveSingleLeg
from controller.PartialMovements.RepositionSingleLegs import RepositionSingleLegs

#from SpeechModule import SpeechModule
#from Restrictedness import WorkspaceRestrictedness as WorkspaceRestrictedness
import geomparse

import time
##  Getting the command line arguments. 
def _args():
    import argparse
    parser = argparse.ArgumentParser(description="WALKNET start skript ")
    parser.add_argument("-t", "--simulationDuration", action="store", default=float('inf'), type=float,
        dest="simulationDuration", help="Duration of the simulation in seconds (in simulation time).")
    parser.add_argument("-l", "--log", action="store_true", default=False,
        dest="log", help="Log output.")
    parser.add_argument("-s", "--Startposture", action="store", default=0, type=int,
        dest="startPostureNumber", help="Configuration of initial posture (given for each leg as number from 0 to 4)")
    return parser.parse_args()

##  Initialisation of the environment.
#   Building up the communication client and starting the main loop.
def init_environment(start_posture_number, star_posture):

    randomLongStep = [5.4, [2]] # exp #my_list[0] Exp 5.84 3 4 = 894 Exp 3.53 3 4 = 
    randomLongStep_time = int(100*randomLongStep[0])
    randomLongStep_leg = randomLongStep[1]
    
    simulation_duration = 24
    controllerFrequency=100 # Define the frequency with which the controller should run.

    # Create a communication client for the timing of the simulation
    simServ=communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])

    # Setup the robot and its environment in the simulation
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    path = os.path.dirname(os.path.abspath(filename))

    # Send the geometry of the universe and the robot to the simulation.
    simServ.geometryXml=open(path+'/../../GeometryXmls/UniverseWithPlane.xml','r').read()
    simServ.geometryXml=open(path+'/../../GeometryXmls/StartingPosture2.xml','r').read()
    #simServ.geometryXml=open(path+'/GeometryXmls/PolePosition.xml','r').read()
    geometryData = geomparse.parseHectorXml(open(path+'/../../GeometryXmls/Hector.xml','r').read())
    # Create a communication client for the inertial measurement unit
    if not(WSTATIC.real_robot):
        imu=communication_interface.CreateBfbClient(120, ["IMU_PROT"])
    
    # Module main executor
    from ProcessOrganisation.ProcessModule.ProcessModuleQueuedExecution import ProcessModuleQueuedExecution
    mainProcessModuleExecution = ProcessModuleQueuedExecution(debug_time=False)
    
    ########### Build up the individual Controller Modules

    # Build up the wRobot - connects to the simulator
    robot = Robot("Robot", geometryData, communication_interface)
    mainProcessModuleExecution.add_module(robot)

    wRobot = WRobot(robot)
    
    from controller.reaCog.CognitiveExpansion.SwitchRobot import SwitchRobot
    switchRobot = SwitchRobot(wRobot)
    mainProcessModuleExecution.add_module(switchRobot)
    
#    from ProcessOrganisation.PostureVariationModule import PostureVariationModule
 #   postVar = PostureVariationModule("posture_variation", wRobot, [2,3,3,2,2,3])
  #  mainProcessModuleExecution.add_control_module_to_queue(postVar, 4.)
    if start_posture_number>0:
        from ProcessOrganisation.StartPostureModule import StartPostureModule
        postVar = StartPostureModule("init_posture", wRobot, star_posture)
        mainProcessModuleExecution.add_control_module_to_queue(postVar, 1.8)
    
#ROBOT    mainProcessModuleExecution.add_control_module_to_queue(WaitForInput('WaitForInput',robot), 1)
    mainProcessModuleExecution.add_control_module_to_queue(AdjustHeightOfRobot('AdjustHeightOfRobot', robot, 0.1, target_height=WSTATIC.stanceheight), 1)
#ROBOT    mainProcessModuleExecution.add_control_module_to_queue(WaitForInput('WaitForInput',robot), 1)

    # Create an interface that is able to receive commands via a ZeroMQ-connection
    #external_control_interface=ExternalControlInterface(port_num=5556)
    #mainProcessModuleExecution.add_module(external_control_interface)
    #external_control_interface.addControllerEventCallbackFunction(mainProcessModuleExecution.forwardMessageToCurrentController)


    # 1. The walknet controller
    walknet = Walknet("walknet", switchRobot)
    mainProcessModuleExecution.add_control_module_to_queue(walknet, float('Inf'))
    #walknet.setVocalizationMethod(speech_module.vocalizeMessage)
    switchRobot.set_motivation_net(walknet.motivationNetRobot)

    # Load the module for the timing of the simulator
    simulatorTimer = SimulatorTimerModule("Simulator_Timer", robot, controllerFrequency)
    mainProcessModuleExecution.add_module(simulatorTimer)
    
    # Visualizations taking data from the body model
    # a) Single leg position plot over time including PEP
#    from controller.reaCog.Visualization.PyPlotData.PositionVisualizationModule import PositionVisualizationModule
 #   positionVisualization = PositionVisualizationModule("positionVisualization", walknet.motivationNetRobot)
  #  mainProcessModuleExecution.add_module(positionVisualization)

    # b) Single joint angle visualization over time (target and real joint angle)
#    from controller.reaCog.Visualization.PyPlotData.SingleJointAngleVisualizationModule import SingleJointAngleVisualizationModule
#    jointVisualization = SingleJointAngleVisualizationModule("jointVisualization", wRobot)
#    mainProcessModuleExecution.add_module(jointVisualization)
    
#    from controller.reaCog.Visualization.PyPlotData.CoGVisualizationModule import CoGVisualizationModule
 #   cogVisualization = CoGVisualizationModule("cogVisualization", walknet.motivationNetRobot.bodyModelStance)
  #  mainProcessModuleExecution.add_module(cogVisualization)
    
    # d) footfall patterns plotted over time
#    from controller.reaCog.Visualization.PyPlotData.PEPVisualizationModuleInternalSimulation import PEPVisualizationModule
 #   pepVisualization = PEPVisualizationModule("pepVisualization", walknet.motivationNetRobot)
  #  mainProcessModuleExecution.add_module(pepVisualization)
#    from controller.reaCog.Visualization.PyPlotData.PEPVisualizationModuleIsSwinging import PEPVisualizationModule
 #   pepVisualization = PEPVisualizationModule("pepVisualization", walknet.motivationNetRobot)
  #  mainProcessModuleExecution.add_module(pepVisualization)
    
    # e) BodyModel visualization - top view in 2D
#    from controller.reaCog.Visualization.PyPlotData.BodyModel2DViewModuleProbehandeln import BodyModel2DViewModule
#   bm2DVisualization = BodyModel2DViewModule("bodymodel2DVisualization", walknet.motivationNetRobot.bodyModelStance) # use "bm" for visualization of the planning model
#    bm2DVisualization = BodyModel2DViewModule("bodymodel2DVisualization", switchRobot.mmcRobot, walknet.motivationNetRobot) # use "bm" for visualization of the planning model
#    mainProcessModuleExecution.add_module(bm2DVisualization)
    # f) BodyModel visualization - view in 3D
    #from controller.reaCog.Visualization.PyPlotData.BodyModel3DViewModule import BodyModel3DViewModule
    #bm3DVisualization = BodyModel3DViewModule("bodymodel3DVisualization", walknet.motivationNetRobot.bodyModelStance)
    #mainProcessModuleExecution.add_module(bm3DVisualization)

    # g) Online Visualization of the Motivation network
#   from controller.reaCog.Visualization.MotivationNetVisualization.TKCompleteNet import TKCompleteNet
#   motivationNetVisualization = TKCompleteNet("motivationNetVisualization", walknet.motivationNetRobot)
#   mainProcessModuleExecution.add_module(motivationNetVisualization)
    
    from controller.reaCog.Visualization.PyPlotData.DataCollector import DataCollector
    if not(WSTATIC.real_robot):
        dataCollector = DataCollector("DataCollector", walknet.motivationNetRobot, imu, ("posture_var_" + str(start_posture_number).zfill(4)), simulation_duration)
    else: 
        dataCollector = DataCollector("DataCollector", walknet.motivationNetRobot, None, ("posture_var_" + str(start_posture_number).zfill(4)), simulation_duration)
    mainProcessModuleExecution.add_module(dataCollector)
#    from controller.reaCog.Visualization.PyPlotData.DataCollectorMU import DataCollectorMU
 #   dataCollectorMU = DataCollectorMU("DataCollectorMU", walknet.motivationNetRobot, ("posture_var_" + str(start_posture_number).zfill(4)) )
  #  mainProcessModuleExecution.add_module(dataCollectorMU)
    
    # This is part of the hack that makes sure all data is available at the begin of a control cycle.
    joints=[]
    for leg_num, leg in enumerate(robot.legs):
        if RSTATIC.legs_enabled[leg_num]:
            for joint in leg.joints:
                joints.append(joint)
                
    #log_file_name='log_file'+str(random.randint(0,10**10))+'.txt'
    
    log_max_thrsh_back = -1.
    log_min_thrsh_front = 1.
    log_back_time = 0
    log_front_time = 0
    
    try:    
        simulationTime = 0 # In order to let the simulation run in realtime, the simulation time is tracked and compared to the real time in order to let the simulation sleep if it's too fast.
        mainProcessModuleExecution.init_all_modules()
        it = 0
        
        #from controller.reaCog.MotivationNetwork.MotivationUnit import saveMotivationNetToDot
        #saveMotivationNetToDot()
        
        # THE MAIN LOOP
        while (simulationTime <= simulation_duration):
            # The following block is a hack that makes sure only one request is sent to a client at once. 
            for joint in joints:
                joint.UpdateValueIfTooOld('inputPosition')
            for joint in joints:
                joint.UpdateValueIfTooOldAndWait('inputPosition')
#           time.sleep(1/10000)
            for joint in joints:
                joint.UpdateValueIfTooOld('torsion')
            for joint in joints:
                joint.UpdateValueIfTooOldAndWait('torsion')
#           time.sleep(1/10000)
            for joint in joints:
                joint.UpdateValueIfTooOld('outputPosition')
            for joint in joints:
                joint.UpdateValueIfTooOldAndWait('outputPosition')
            
            #log_file=open(log_file_name, 'a')
            #log_file.write('time: '+str(simulationTime)+'\n')
            #for joint in joints:
            #   log_file.write(hex(joint.GetBioFlexBusId())+': '+ str(joint.inputPosition)+', '+str(joint.torsion)+', '+str(joint.outputPosition)+'\n')
            
            #log_file.close()
            if __debug__:
                if not(WSTATIC.real_robot):
                    print("The position of the robot is: ", [round(i,2) for i in imu.position])
                    print("The rotation of the robot is: ", [round(i,2) for i in imu.rotation])
                    print("The magneticField sensed by the robot is: ", [round(i,2) for i in imu.magneticField])
                    print("The acceleration sensed by the robot is: ", [round(i,2) for i in imu.acceleration])
                                
            if (it % 100 == 1):
                print(simulationTime, "- switch decoupled = ", switchRobot.switch.decoupled)
                
                #for i in range(len(switchRobot.mmcRobot.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.original_MUs)):
                 #   print(i, " - SAL: ", switchRobot.mmcRobot.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_SAL[i].output_value, 
                  #      " ; WTA: ", switchRobot.mmcRobot.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value,
                   #     " ; RTB: ", switchRobot.mmcRobot.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_RTB[i].output_value)
                #os.system("screencapture sim_" + str(it) + ".png")
            #print(mainProcessModuleExecution.current_control_module.name)
            if mainProcessModuleExecution.current_control_module.name==walknet.name:
                #print('debug1')
                it += 1

                if (it == randomLongStep_time):
                    print('INTRODUCE DISTURBANCE')
                    for leg_nr in randomLongStep_leg:
                        walknet.motivationNetRobot.motivationNetLegs[leg_nr].moveNextSwingToFront(numpy.array([0.15, 0., 0.]))
            
#            if (it == 380):
#               old_ml_aep = numpy.array(walknet.motivationNetRobot.middle_left_leg.aep)
#                old_hl_aep = numpy.array(walknet.motivationNetRobot.hind_right_leg.aep)
#               walknet.motivationNetRobot.middle_left_leg.aep += numpy.array([0.05, 0., 0.])
#                walknet.motivationNetRobot.hind_right_leg.aep += numpy.array([0.05, 0., 0.])
#                print("Swing movement further to anterior")
#            if (it == 550):
#               walknet.motivationNetRobot.middle_left_leg.aep = old_ml_aep
#                walknet.motivationNetRobot.hind_right_leg.aep = old_hl_aep
#                print("Swing movement AEP reset")

            mainProcessModuleExecution.execute_complete_step(simulationTime)
            
            simulationTime+=1/controllerFrequency # Update the variable holding the current simulation time.
            communication_interface.NotifyOfNextIteration() # Tell the communication interface that a new controller iteration has begun.
            #print("current " + str(walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back) + str(log_max_thrsh_back) + str(walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back > log_max_thrsh_back) )
            if (it > 100):
                if (walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back > log_max_thrsh_back):
                    log_max_thrsh_back = walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back
                    log_back_time = simulationTime
                    #print("current " + str(walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back))
                if (walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_front < log_min_thrsh_front):
                    log_min_thrsh_front = walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_front
                    log_front_time = simulationTime
                    #print("current " + str(walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_front) + " / " +str(simulationTime) )
#    log_front_time = 0
            
            
        # Write down a summary of the simulation run
        log_file=open('sim_summary.txt', 'a')
        log_file.write('Exp ' + str(start_posture_number) + ", max back stability = " + str(log_max_thrsh_back) 
            + " at: " + str(log_back_time) +
            ", min front stability = " + str(log_min_thrsh_front) 
            + " at: " + str(log_front_time) + '\n')
        #str(walknet.motivationNetRobot.pD_unstableCheck_HL.iteration_instable) + str(walknet.motivationNetRobot.pD_unstableCheck_HR.iteration_instable) +'\n')
        #log_file.write('time: '+str(simulationTime)+'\n')
        #for joint in joints:
        #   log_file.write(hex(joint.GetBioFlexBusId())+': '+ str(joint.inputPosition)+', '+str(joint.torsion)+', '+str(joint.outputPosition)+'\n')
        
    except KeyboardInterrupt as err:
        print("\n terminated by user", err)

## Main method
if __name__ == "__main__":
    if sys.version_info < (3, 2):
        print("This programm requests Python 3.2 or higher")
        quit()
    args = _args()
    #init_environment(args.startPostureNumber)
    import pickle
    with open('vary_starting_posture_exp_A', 'rb') as f:
        start_posture_list = pickle.load(f)

    start_posture = args.startPostureNumber
    #start_posture = 1687#unfinished_runs[args.startPostureNumber]
    # Create the communication interface
    communication_interface=comminter.CommunicationInterface()
    protocolXmlDirectory="../hector/BioFlexBusProtocolXmls/"
    communication_interface.ParseProtocolXmls([protocolXmlDirectory+protocolXml for protocolXml in ["BIOFLEX_1_PROT.xml", "BIOFLEX_ROTATORY_1_PROT.xml", "BIOFLEX_ROTATORY_CONTROL_1_PROT.xml", "BIOFLEX_ROTATORY_ERROR_PROT.xml", "SIMSERV_1_PROT.xml", "IMU_PROT.xml", "PRESSURE_SENSOR_PROT.xml"]]) # Parse the xml files that contain the command-protocol definitions
    
    #for posture_number in range(0, start_posture_list.shape[0]):
    print("*************************************")
    print("Start experiment : " + str(start_posture) + " - " + str(start_posture_list[start_posture]))
    print("*************************************")
    init_environment(start_posture, start_posture_list[start_posture])
    
    # Plot the Data
    from controller.DataPlayer.PEPVisualizationPlayer_allPhases import PEPVisualizationPlayer
    pepVisualization = PEPVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/logs/reacog_walking_" + "posture_var_" + str(start_posture).zfill(4) )

    """"# Plot the Data
    from controller.DataPlayer.PositionVisualizationPlayer import PositionVisualizationPlayer
    posVisualization = PositionVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/" + randomLongStep_name + '_all_data.out' )
    # Plot the Data
    from controller.DataPlayer.CoGVisualizationPlayer import CoGVisualizationPlayer
    cogVisualization = CoGVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/" + randomLongStep_name + '_all_data.out' )
    print("Zeit: ", (time.clock() - before) )
    input()
    """
