Currently working with the timed robot setup and hope to convert it to a command based setup once it's done.


If for some reason you use this code over better code than here's the general gist of how I currently envision it working once finished:

  --The roboRio will handle motor outputs as well as communicating to the laptop and raspberry pi the current phase (robotInit, teleopPeriodic, etc.). I think I'll also have it send encoder values to the laptop.
  
  --The raspberry pi will handle putting the camera feed to the dashboard as well as processing the images (likely using GRIP generate code) for the AprilTag coordinates. I'll have to double check but I think last year I came up with some math for getting the robot position using the corners, but point is it'll communicate to the laptop if it was able to obtain the robot coordinates using the cameras. In addition to all this it will have a navX linked to an I2C port and will be sending updates about the x, y, and z position, velocity, and acceleration as well as the quaternion representation of the robot.  

  --The laptop will receive the encoder values, navX values, and image coordinates (if there are any) and evaluate what the robot position, angle, etc. most likely is. It will use this as well as the current phase, game specific information (e.g. I think I currently have a cone/I think I currently don't have a cone) to generate a trajectory for the robot to follow. If this trajectory is estimated to deviate too much from the current trajectory then it will send the new trajectory to the roboRio.


Right now the code is very messy and uncommented, and this is because I plan to reorganize it once finished, at which point I'll comment stuff and rename things if needed.
If you have questions ask, on the bold assumption I can understand my code I'll be able to answer them.
