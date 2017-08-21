## FTC Vuforia Vision

### Part 1: The Drive Class

Why use this library:
1.	Vector based
    -	Many sensors report position as coordinates in X-Y space relative to some target. For instance Vuforia reports distance and offset from a beacon. These coordinates are the vector pointing to the target which typically needs to be converted to polar coordinates to control standard Mecanum drive vehicles. The advantage of this class is that it takes a direction vector as an input, thus avoiding extra calculations.
2. Optimized predictive Turn Controller
    -	Turn controller can be configured to provide the fastest turn speeds by predicting robot position and avoiding oversteering. The controller can minimize oscillations while turning and prevent overcorrecting multiple times.
3.	Voltage Drop Correction
    -	Library has the ability to increase the assigned power if the voltage drops, thus providing more predictable behavior with low battery. 


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
Experimenting with encoders on a mecanum drived showed that the encoders could not be reliably used for mecanum driving. Additionally, it showed that simply turning and then driving straight is faster than attempting to use mecanum capabilities. The best use of sideways driving is with the VecDrive methods across very small distances where you have access to supporting data from sensors and not just encoders. 
| Method | Description |
|-----------------------------|------------------------|
|`void DriveByEncoders(double heading, double power, int distance)`| This method is best used for driving across large distances. It does not use mecanum capabilities, that is, maintaining current robot orientation while moving in any direction. Instead, the robot simply rotates to the specified heading as it moves forward. There are three parameters: **heading**, which is measured from -180 to 180, with 0 being the front of the robot during opmod initialization, **power**, the speed with which you want the robot to go, **distance** which is derived from encoders. It is an arbitrary unit that varies across robots. This method will block calling thread until the specified distance is reached according data from *encoders*. If a new command is not sent after it returns, the robot will continue with the same heading and power. This is done to ensure smooth transitions between commands. |
|`void TurnToAngle(double heading)`| Turns in place to a specified angle. |
|`void VecDrive(double x, double y, double power, int maxDuration)`|Drives in the diretion pointed by the vector with specified power for specified amount of time. X is forwards, Y is side to side, relative to the current robot heading. This method **will** use Mecanum drive capabilities, while maintaining robot orientation relative to the field. Best for use for small adjustments in front of your target. This method **does not block** calling thread, this is needed to allow for constant check on its position relative to the target. When new command is issued, it immediately cancels the current one. maxDuration limits how long it will run if no other commands is recieved to prevent robot running away if something goes wrong.|
|`void VecDriveBalanced(double x, double y, double power, int maxDuration)`|Same as VecDrive, but uses Voltage Correction to account for voltage drop.|
|`void ReverseDirection()`|Can be used to configure motors so that it functions as a proper mecanum drive system.|
|`void brake()`| Used to stop the robot.|
#### Example Implementation: Move straight, move at an angle then stop and rotate in place.
```Java
drive.DriveByEncoders(0,1,200); //Moves straight for 200 units with a power setting of 1.
drive.DriveByEncoders(45,1,500); //Without stopping, continues turning 45 degrees to the right for 500 units with a power setting of 1.
drive.brake(); //Stops 
drive.TurnToAngle(180); //Rotates to 180 degrees, facing the direction opposite to what it started with.
```

#### Image depicting what the above code will make the robot do:
![Path](http://i.imgur.com/kygFhsu.png)

1. Description
    1.	The drive class is one that is in charge of regulating the motor power levels. It can take information from sensors, encoders and others. The ones built in are nav-x (w/ pid controller for rotation) and encoders for distance travel.
    2.	The drive class has its own thread which continuously updates the motor values based on new information.
    3.	The nav-x is its own separate thread which blocks the drive thread until new information is received. See the flowchart for a clearer picture.
2.	Usage:
    1.	Initialize drive motors. Do not assign directions here.
    2.	Initialize nav_x device.
    3.	Construct drive class (only on constructor available)
    4.	If needed, use ReverseDirection to reverse motor directions.
    5.	From opmode class, call different drive methods to control the motor power values.
3.	Configuration
    1.	The Drive class is very versatile and can be customized for your needs.
    2.	Adjustable class constants:
        -	YAW_PID_P – this constant regulates the PID controller for rotation, greater values result in stronger, faster rotations. We used .013, this will likely differ.
        -   Tolerance degrees, how close to the desired angle you want the robot to run to. Default = 2.0
        -	MIN/MAX Motor output values – change if you wish to reduce the maximum speed of the robot. Default -1/1
        -	GYRO_DEVICE_TIMEOUT_MS – Default 500, for navx setup. Change not recommended
    3.	In constructor:
        -   One of the most important changeable elements is the rotation PID controller constants, it allows you to adjust how when to stop giving power to the drive train or when to counteract overcorrection. 
        -	These values can be changed from the yawPIDController constructor.
        -	The elements are as follows: navx device, Static Rotation Rules, Moving Rotation Rules, Overcorrected Rotation rules, and “Log” Boolean (default true)
            1.	Static Rotation Rules is a 3x# array of rules:
                
                | Error, in degrees | Angular Velocity Degrees/Milliseconds | Correction Desired. ‘Dump’ |
                |-----------------------------|------------------------|---------|
                |##|##|##|
                
                The program will check two things, first it checks if the error is smaller than the designated error, then it will check that the angular velocity is GREATER than the designated AV. If this is true, it will set the dump value, to the designated dump value, which is usually zero in this case because you would like the robot to stop moving. If the conditions are not met, it will move onto the next row and check if any other conditions are met. If none are met, it reverts to the default dump value and uses whatever correction the PID controller demands based on the current angle of the robot.
         
             2.	Moving Rotation Rules
                This is for situations where the robot is moving in a line but is also needing to turn at an angle. The checks are the same as those for the static rotation rules. However, it is recommended to use a negative dump value… ?
          
             3.	Finally is the Overcorrected Rotation Rules, which regulates how strong you want the reaction to overcorrection to be. The same checks apply. Use a positive dump value… ?
             4.	The log is for constructor purposes only.
    4.	In the class you have several other options, which includes creating drive modes – for instance, you can create a new one that includes some other corrective variable. One example is the VecDriveBalanced included mode, which incorporates the voltage and gets voltage coefficients that adjust the returned power values if there has been a voltage drop. Another example is changing the scaling on the x-value in VecDrive/VecDriveBalanced. Currently it is scaled down by 2 because the robot goes robot forwards faster than it does sideways.  This correction makes similar x and y values give similar speeds. (x = forward and backward from center of robot). This will likely differ for your robot, especially if you do not use mecanum drive.
    5.	An example of a method you can create is a test method which will power up one motor at a time.

#### Flowchart displaying the function of the Drive Class in conjunction with the NavX
![Flowchart](http://i.imgur.com/5u3rdjM.png)
