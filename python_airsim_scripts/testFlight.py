import modules  
import time
# Get the drone client object  
client = modules.get_client()  
  

  

modules.take_Off(client)  


modules.go_Up(client, 2) 
time.sleep(1)

client.rotateToYawAsync(10,5,1).join() #rotate yaw at 10 degrees for 5 seconds
client.rotateToYawAsync(15,4,1).join()
client.rotateToYawAsync(-10,5,1).join()
client.moveByVelocityAsync(1,1,0,2).join() #move on x and y plane for 1 meter for 2 seconds

client.moveByVelocityAsync(1,-1,0,4).join()
client.hoverAsync(100).join()

#move by velocity(forward, right, down, duration)