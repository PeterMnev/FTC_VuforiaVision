## FTC Vuforia Vision

### Part 1: The Drive Class

Why use this library:
1.	Vector based
    -	Many sensors report position as coordinates in X-Y space relative to some target. For instance Vuforia reports distance and offset from a beacon. These coordinates are the vector pointing to the target which typically needs to be converted to polar coordinates to control standard Mecanum drive vehicles. The advantage of this class is that it takes a direction vector as an input, thus avoiding extra calculations.
2.	Voltage Drop Correction
    -	Library has the ability to increase the assigned power if the voltage drops, thus providing more predictable behavior with low battery.
3.	Optimized predictive Turn Controller
    -	Turn controller can be configured to provide the fastest turn speeds by predicting robot position and avoiding oversteering. The controller can minimize oscillations while turning and prevent overcorrecting multiple times. 


### Drive Class Declaration

#### Constructor
``` Java
Drive(DcMotor frontLeftMotor, 
DcMotor frontRightMotor, 
DcMotor backLeftMotor, 
DcMotor backRightMotor, 
AHRS navXDevice, 
Telemetry telemetry, 
LinearOpMode opMode)
```
Creates a new Drive object

#### Methods

| Method | Description |
|-----------------------------|------------------------|
|void DriveByEncoders(double heading, double power, int distance)| Drives a specified distance (arbitrary units, vary from robot to robot – depend on drivetrain) in the specified direction (angle measured from -180 to 180, 0 is the front of the robot), at specified power.|
|void TurnToAngle(double heading)| Turns in place to a specified angle. |
|void VecDrive(double x, double y, double power, int maxDuration)|Drives on a vector with specified power for specified amount of time. X is forwards, Y is side to side. Use in conjunction with sensor e.g. Vuforia to provide constant updates, or set a specified vector and then a wait statement to maintain that vector. Make sure the wait statement does not exceed maxDuration, or you will simply get the maxDuration.|
|void VecDriveBalanced(double x, double y, double power, int maxDuration)|Drives on a vector with specified power for specified amount of time. Uses Voltage Correction to account for voltage drop. Usage same as VecDrive.|
|void ReverseDirection()|Can be used to configure motors so that it functions as a proper mecanum drive system.|


i.	The drive class is one that is in charge of regulating the motor power levels. It can take information from sensors, encoders and others. The ones built in are nav-x (w/ pid controller for rotation) and encoders for distance travel.
ii.	The drive class has its own thread which continuously updates the motor values based on new information.
iii.	The nav-x is its own separate thread which blocks the drive thread until new information is received. See the flowchart for a clearer picture.
c.	Usage:
i.	Initialize drive motors. Do not assign directions here.
ii.	Initialize nav_x device.
iii.	Construct drive class (only on constructor available)
iv.	If needed, use ReverseDirection to reverse motor directions.
v.	From opmode class, call different drive methods to control the motor power values.
d.	Configuration
i.	The Drive class is very versatile and can be customized for your needs.
ii.	Adjustable class constants:
1.	YAW_PID_P – this constant regulates the PID controller for rotation, greater values result in stronger, faster rotations. We used .013, this will likely differ.
2.	Tolerance degrees, how close to the desired angle you want the robot to run to. Default = 2.0
3.	MIN/MAX Motor output values – change if you wish to reduce the maximum speed of the robot. Default -1/1
4.	GYRO_DEVICE_TIMEOUT_MS – Default 500, for navx setup. Change not recommended
iii.	In constructor:
1.	One of the most important changeable elements is the rotation PID controller constants, it allows you to adjust how when to stop giving power to the drive train or when to counteract overcorrection. 
2.	These values can be changed from the yawPIDController constructor.
3.	The elements are as follows: navx device, Static Rotation Rules, Moving Rotation Rules, Overcorrected Rotation rules, and “Log” Boolean (default true)
a.	Static Rotation Rules is a 3x# array of rules:
Error, in degrees	Angular Velocity
Degrees/Milliseconds	Correction Desired. ‘Dump’
The program will check two things, first it checks if the error is smaller than the designated error, then it will check that the angular velocity is GREATER than the designated AV. If this is true, it will set the dump value, to the designated dump value, which is usually zero in this case because you would like the robot to stop moving. If the conditions are not met, it will move onto the next row and check if any other conditions are met.
If none are met, it reverts to the default dump value and uses whatever correction the PID controller demands based on the current angle of the robot.
b.	Moving Rotation Rules
This is for situations where the robot is moving in a line but is also needing to turn at an angle. The checks are the same as those for the static rotation rules. However, it is recommended to use a negative dump value… ?
c.	Finally is the Overcorrected Rotation Rules, which regulates how strong you want the reaction to overcorrection to be. The same checks apply. Use a positive dump value… ?
d.	The log is for constructor purposes only.
iv.	In the class you have several other options, which includes creating drive modes – for instance, you can create a new one that includes some other corrective variable. One example is the VecDriveBalanced included mode, which incorporates the voltage and gets voltage coefficients that adjust the returned power values if there has been a voltage drop. Another example is changing the scaling on the x-value in VecDrive/VecDriveBalanced. Currently it is scaled down by 2 because the robot goes robot forwards faster than it does sideways.  This correction makes similar x and y values give similar speeds. (x = forward and backward from center of robot). This will likely differ for your robot, especially if you do not use mecanum drive.
v.	An example of a method you can create is a test method which will power up one motor at a time.

![Text](http://i.imgur.com/5u3rdjM.png)
