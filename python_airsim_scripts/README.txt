
##IN CASE YOU SKIPPED THE DOCUMENTATION

REQUIREMENTS - 
1) A conda environment based upon the libraries listed in environment.yml
2) AirSim installed (from Colosseum), running on UE.
3) ZED SDK installed


follow_object_airsim.py - Connects the zed camera on a laptop, and then to an AIrSim client. The zed will than employ the human detector network,
capture the first person, read the distance, and then target the drone to move toward that distance in meters (AirSim default distane scale).
Along the xyz axis we fixed the y axis(tilt up or down) in the demo to keep the drone from iterating to the ground. 
You can uncomment line 126 and add the more instruction based logic you wish the drone to follow after reaching the waypoint.


airsim_toROS.py - The module code that converts the AirSim API to ROS interpretable commands. (put this in a node directory, build the node, then run)
**TO_DO** - Create a ROS node in the follow_person_airsim.py that interprets the ROS code to incorporate communication between other flight modules needed for real world flight(can simulate the flight modules as well)


follow_path_airsim.py - Experimental code for passing the trajectory (path) from a ROS Subscribor node the AirSim movements. 

modules.py - A simple set of AirSim flight methods - incorporates with out without ROS connection (check line 15)

testFlight.py - simple test flight script to confirm compatibility 






On windows, if you are installing Unreal Engine, you will need to install Visual Studio's .NET and c++ toolkits first. 
Here is a detailed video of that process - https://www.youtube.com/watch?v=8xJRr6Yr_LU

It is also important to note that Microsoft stopped supporting AirSim, so you must pull from this open source version from Colosseum https://github.com/CodexLabsLLC/Colosseum
Much of the documentation is identical to the AirSim documentation, and the plugin for UE is installed the same.
The proper AirSim documentation can be found here https://microsoft.github.io/AirSim/build_windows/
For our Airsim environment, it is built with UE5.2 on Windows 10. Our AirSimMap requires UE5.2.
https://www.youtube.com/watch?v=1oY8Qu5maQQ&t=1s this video goes through the process of building airsim in UE from scratch (after installing UE5)

In order to run any of the scripts which connect to AirSim, you need to first run the AirSimMap, click the green play button, and click 'no' for MotorVehile AirSim.
Then run the python script and the program should connect to the UE5 client automatically. 


