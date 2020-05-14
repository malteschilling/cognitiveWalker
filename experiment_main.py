import inspect, os
import numpy
from controller.reaCog.walknet.walknetF import Walknet
from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import SimulatorTimerModule as SimulatorTimerModule
from Hector.RobotF import Robot
from controller.reaCog.walknet.WRobotF import WRobot

from Hector import RobotSettings as RSTATIC
from controller.reaCog import WalknetSettings as WSTATIC
from controller.PartialMovements.WaitForInput import WaitForInput
from controller.PartialMovements.AdjustHeightOfRobot import AdjustHeightOfRobot

import geomparse

##  Initialisation of the environment.
#   Building up the communication client and starting the main loop.
def init_environment(communication_interface, start_posture_number, disturbed_step=[], simulation_duration=20):
    
    if (len(disturbed_step) > 1):
        randomLongStep_time = int(100*disturbed_step[0])
        randomLongStep_leg = disturbed_step[1]
    else:
        randomLongStep_time = -1
    
    controllerFrequency=100 # Define the frequency with which the controller should run.

    # Create a communication client for the timing of the simulation
    simServ=communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])

    # Setup the robot and its environment in the simulation
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    path = os.path.dirname(os.path.abspath(filename))

    # Send the geometry of the universe and the robot to the simulation.
    simServ.geometryXml=open(path+'/GeometryXmls/UniverseWithPlane.xml','r').read()
    simServ.geometryXml=open(path+'/GeometryXmls/StartingPosture2.xml','r').read()
    geometryData = geomparse.parseHectorXml(open(path+'/GeometryXmls/Hector.xml','r').read())
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
    
    if start_posture_number>0:
        import pickle
        with open('vary_starting_posture_exp_A', 'rb') as f:
            start_posture_list = pickle.load(f)
        from ProcessOrganisation.StartPostureModule import StartPostureModule
        postVar = StartPostureModule("init_posture", wRobot, start_posture_list[start_posture_number])
        mainProcessModuleExecution.add_control_module_to_queue(postVar, 1.8)
    
    # The next line is required for the real robot only: wait until robot is setup.
	# mainProcessModuleExecution.add_control_module_to_queue(WaitForInput('WaitForInput',robot), 1)
    mainProcessModuleExecution.add_control_module_to_queue(AdjustHeightOfRobot('AdjustHeightOfRobot', robot, 0.1, target_height=WSTATIC.stanceheight), 1)
    # The next line is required for the real robot only: another wait until experiment
    # can be started (cameras rolling ...)	
	#mainProcessModuleExecution.add_control_module_to_queue(WaitForInput('WaitForInput',robot), 1)

    # 1. The walknet controller
    walknet = Walknet("walknet", switchRobot)
    mainProcessModuleExecution.add_control_module_to_queue(walknet, float('Inf'))
    switchRobot.set_motivation_net(walknet.motivationNetRobot)

    # Load the module for the timing of the simulator
    simulatorTimer = SimulatorTimerModule("Simulator_Timer", robot, controllerFrequency)
    mainProcessModuleExecution.add_module(simulatorTimer)
    
    # Different visualizations can be turned on.
    # Furthermore: after experiments, data can be plotted (see single experimental calls).
    # BodyModel visualization - top view in 2D
    #from Visualization.BodyModel2DViewModuleProbehandeln import BodyModel2DViewModule
    #bm2DVisualization = BodyModel2DViewModule("bodymodel2DVisualization", switchRobot.mmcRobot, walknet.motivationNetRobot) # use "bm" for visualization of the planning model
    #mainProcessModuleExecution.add_module(bm2DVisualization)


    # Online Visualization of the Motivation network
    #from Visualization.TKCompleteNet import TKCompleteNet
    #motivationNetVisualization = TKCompleteNet("motivationNetVisualization", walknet.motivationNetRobot)
    #mainProcessModuleExecution.add_module(motivationNetVisualization)
    
    from Visualization.DataCollector import DataCollector
    if not(WSTATIC.real_robot):
        dataCollector = DataCollector("DataCollector", walknet.motivationNetRobot, imu, ("posture_var_" + str(start_posture_number).zfill(4)), simulation_duration)
    else: 
        dataCollector = DataCollector("DataCollector", walknet.motivationNetRobot, None, ("posture_var_" + str(start_posture_number).zfill(4)), simulation_duration)
    mainProcessModuleExecution.add_module(dataCollector)
    
    # Turn this on for recording activation of all neurons in the motivation unit network.
    from Visualization.DataCollectorMU import DataCollectorMU
    dataCollectorMU = DataCollectorMU("DataCollectorMU", walknet.motivationNetRobot, ("posture_var_" + str(start_posture_number).zfill(4)) )
    mainProcessModuleExecution.add_module(dataCollectorMU)
    
    # In order to assure that the data structures are available during first iteration,
    # these are explicitly created once.
    joints=[]
    for leg_num, leg in enumerate(robot.legs):
        if RSTATIC.legs_enabled[leg_num]:
            for joint in leg.joints:
                joints.append(joint)
    
    log_max_thrsh_back = -1.
    log_min_thrsh_front = 1.
    log_back_time = 0
    log_front_time = 0
    
    try:    
        simulationTime = 0 # In order to let the simulation run in realtime, the simulation time is tracked and compared to the real time in order to let the simulation sleep if it's too fast.
        mainProcessModuleExecution.init_all_modules()
        it = 0
        
        # The motivation net structure can be written to a .dot file for visualization.
        #from controller.reaCog.MotivationNetwork.MotivationUnit import saveMotivationNetToDot
        #saveMotivationNetToDot()
        
        # THE MAIN LOOP
        while (simulationTime <= simulation_duration):
            # Iterate over all variables that have to be accessed through the BioFlexBus 
            # client in order to update these. Explicit call guarantees that 
            # only one request is sent to a client at once. 
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
            
            if __debug__:
                if not(WSTATIC.real_robot):
                    print("The position of the robot is: ", [round(i,2) for i in imu.position])
                    print("The rotation of the robot is: ", [round(i,2) for i in imu.rotation])
                    print("The magneticField sensed by the robot is: ", [round(i,2) for i in imu.magneticField])
                    print("The acceleration sensed by the robot is: ", [round(i,2) for i in imu.acceleration])
                                
            if (it % 100 == 1):
                print(simulationTime, "- switch decoupled = ", switchRobot.switch.decoupled)
                
            if mainProcessModuleExecution.current_control_module.name==walknet.name:
                it += 1

                # Apply disturbance during specific steps.
                # Given is a time - and during the following swing movement for 
                # the specified legs, the swing movement is disturbed to the front
                # (but only for the next swing movement).
                if (it == randomLongStep_time):
                    print('INTRODUCE DISTURBANCE')
                    for leg_nr in randomLongStep_leg:
                        walknet.motivationNetRobot.motivationNetLegs[leg_nr].moveNextSwingToFront(numpy.array([0.15, 0., 0.]))

            mainProcessModuleExecution.execute_complete_step(simulationTime)
            
            simulationTime+=1/controllerFrequency # Update the variable holding the current simulation time.
            communication_interface.NotifyOfNextIteration() # Tell the communication interface that a new controller iteration has begun.

            # Logging of data for evaluation: log when an instable posture was detected.
            if (it > 100):
                if (walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back > log_max_thrsh_back):
                    log_max_thrsh_back = walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back
                    log_back_time = simulationTime
                    #print("current " + str(walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_back))
                if (walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_front < log_min_thrsh_front):
                    log_min_thrsh_front = walknet.motivationNetRobot.bodyModelStance.temp_stability_fact_front
                    log_front_time = simulationTime
            
        # Write down a summary of the simulation run
        # from the logging information - this is appended to the sim_summary.txt file.
        log_file=open('sim_summary.txt', 'a')
        log_file.write('Exp ' + str(start_posture_number) + ", max back stability = " + str(log_max_thrsh_back) 
            + " at: " + str(log_back_time) +
            ", min front stability = " + str(log_min_thrsh_front) 
            + " at: " + str(log_front_time) + '\n')
            
    except KeyboardInterrupt as err:
        print("\n terminated by user", err)

