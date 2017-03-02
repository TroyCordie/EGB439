The code here allows you to control the robot and access its camera from the MATLAB environment.  You need to have the file
```PiBot.m``` in the current directory or on your MATLAB path.  The Raspberry Pi needs to be running the Python server code (```server-camera.py``` and ```server-motor.py```) in the folder above.

We start by connecting to the remote Raspberry Pi
```
pb = PiBot('192.168.1.141');
```
which requires the IP address of the RPi on the network.  This might be the one assigned by the WiFi access point in the 
lab or a hardwired address 192.168.0.100 for the wired ethernet port. 
The result is an object that is a connector to the remote RPi.

To obtain an image is simply
```
img = pb.getImageFromCamera();
```
which is an RGB image with uint8 pixel data.  Typically acquiring an images takes 100-200ms.

To access the motor encoders
```
ticks = pb.getMotorTicks();
```
which returns a 2-vector containing the integer tick counts for motor A and motor B.

To set the speed of the motors is simply
```
pb.setMotorSpeeds(-50,-50);
```
where the two arguments are the speed of motor A and B respectively.  WHAT ARE THE UNITS HERE?

To stop the motors
```
pb.setMotorSpeeds(0,0);
```
