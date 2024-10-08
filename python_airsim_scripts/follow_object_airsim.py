import pyzed.sl as sl
import cv2
import numpy as np
import math
import timeit
import modules
import time
def cal():
    zed = sl.Camera()
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    # Capture 150 images and depth, then stop
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    while True:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)

            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] + #distance formula for Zed camera
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])

            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)

            if not np.isnan(distance) and not np.isinf(distance):
                print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
                zed.close()
            else:
                print("Can't estimate distance at this position.")
                print("Your camera is probably too close to the scene, please move it backwards.\n")
            sys.stdout.flush()
            



def main():
    # Create a Camera object
    zed = sl.Camera()
    
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
        
    main_detection = sl.ObjectDetectionParameters()
    main_detection.enable_tracking=True
    main_detection.image_sync=True
    main_detection.i
    main_detection.detection_model = sl.OBJECT_DETECTION_MODEL.PERSON_HEAD 
    #main_detection.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX  
    camera_infos = zed.get_camera_information()
    if main_detection.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters()
        #positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        zed.enable_positional_tracking(positional_tracking_param)
 
    print("Object Detection: Loading Module...")

    err = zed.enable_object_detection(main_detection)
    if err != sl.ERROR_CODE.SUCCESS :
        print (repr(err))
        zed.close()
        exit(1)
    client = modules.get_client(False)  #Connect to AirSim Client, make TRUE if ROS Environment
    modules.take_Off(client)
    modules.go_Up(client, 2) #Rise 2 meters
    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40
    
    waypoints = [] #Store DRONE YAW Degree rotations from 0-360 here! (adds rightward rotation with increasing values)
    waypoint_tracker = waypoints.__sizeof__

    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        err = zed.retrieve_objects(objects, obj_runtime_param)
        start = timeit.default_timer()
        if objects.is_new :
            obj_array = objects.object_list
            if len(obj_array) > 0 :
                first_object = obj_array[0]
                print("First object attributes:")
                print(" Label '"+repr(first_object.label)+"' (conf. "+str(int(first_object.confidence))+"/100)")
                position = first_object.position
                dimensions = first_object.dimensions
                time.sleep(1)
                client.rotateToYawAsync(0,5,3).join()
                time.sleep(7)
                image = sl.Mat()
                depth = sl.Mat()
                point_cloud = sl.Mat()
                mirror_ref = sl.Transform()
                mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
                tr_np = mirror_ref.m
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve depth map. Depth is aligned on the left image
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud. Point cloud is aligned on the left image.
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                err, point_cloud_value = point_cloud.get_value(x, y)

                distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])
                #print(" 3D position: [{0},{1},{2}]\n 3D dimentions: [{3},{4},{5}]".format(position[0],position[1],position[2],dimensions[0],dimensions[1],dimensions[2]))
                print("3D position: " + str(round(position[2],2)) + "meters from drone")
                print("Angling yaw at " + str(round(15* position[0],2)) + " degrees.") 

                client.rotateToYawAsync(8* position[0],5,3).join() #Turns drone to face object
                client.moveByVelocityAsync(1,position[0],0,position[2] +position[2]*.2).join() #Moves drone to object via the distance received
                #client.rotateToYawAsync(90,5,3).join() do a rotation after moving to object for next waypoint (can have a series of commands here to trigger certain rotattion directions based upon the preplanned path)
                
                #client.rotateToYawAsync([waypoint_tracker],5,3)
                #waypoint_tracker+=1
                ##Uncomment here for sequential waypoint rotations.
                
                point_cloud_np = point_cloud.get_data()
                point_cloud_np.dot(tr_np)
                if not np.isnan(distance) and not np.isinf(distance):
                    print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
                else:
                    pass
                
                print("\n Bounding Box 3D ")
                bounding_box = first_object.bounding_box
                stop = timeit.default_timer()
                print("\n FPS:", stop - start)


    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
