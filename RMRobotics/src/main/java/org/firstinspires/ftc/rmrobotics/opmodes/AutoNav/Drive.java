package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

/**
 * Created by Peter on 1/26/2017.
 */

public class Drive{

    //internal thread
    private static internalThread thrd;
    //runtime calculations
    ElapsedTime runtime = new ElapsedTime();
    // Opmode
    private LinearOpMode opmode = null;

    //navx
    private AHRS navx_device;

    //Telemetry
    Telemetry telemetry;

    // Declaring Navx PID Variables
    private ZPIDController yawPIDController;
    private ZPIDController.PIDResult yawPIDResult;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1;
    private final double YAW_PID_P = 0.013;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private int GYRO_DEVICE_TIMEOUT_MS = 500;


    // Declaring Motors and Voltage Sensor
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private VoltageSensor voltSensor;

    // Declaring encoder counters
    private int prevFLEnc;
    private int prevFREnc;
    private int prevBLEnc;
    private int prevBREnc;


    Drive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, AHRS nx, Telemetry tl, LinearOpMode om)
    {
        //Initializes motors
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        navx_device = nx;
        telemetry = tl;
        opmode = om;

        //set zero behavior and reverse motors so all are in the same direction.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //enable encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.yield();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        voltSensor = (VoltageSensor) frontLeft.getController();

        prevFLEnc = frontLeft.getCurrentPosition();
        prevBLEnc = backLeft.getCurrentPosition();
        prevBREnc = backRight.getCurrentPosition();
        prevFREnc = frontRight.getCurrentPosition();

        //Callibrate NavX - default loop
        boolean calibration_complete = false;
        while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                telemetry.update();
            }
        }

        // Reset yaw
        navx_device.zeroYaw();

        //change whether or not you want the ADB Log to be enabled false = no log
        boolean log = true;

        //initalize a PID controller
        yawPIDController = new ZPIDController(navx_device,
                new double[][] {
                        {24, 0.11, 0},
                        {18, 0.09, 0},
                        {15.5, 0.075, 0},
                        {13, 0.06, 0},
                        {5, 0.01, 0}},
                new double[][] {{20, 0.06, -10}},
                new double[][] {{0, 0.02, 10}},
                log);

        /* Configure yaw PID controller */
        yawPIDController.setSetPoint(0);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE / 2, MAX_MOTOR_OUTPUT_VALUE / 2);
        yawPIDController.setTolerance(TOLERANCE_DEGREES);
        yawPIDController.setP(YAW_PID_P);
        yawPIDController.enable(true);
        yawPIDResult = new ZPIDController.PIDResult();

        //thread init
        thrd = new internalThread();
        new Thread(thrd).start();
    }

    //Requested Vector
//    private  VectorF reqV = new VectorF(0, 0);




    //Difference between reqV and powV





    //First movement method, uses encoders. Takes Angle heading (0 = front, negative = left, up to -180, positive = right up to 180), power 0-1, and distance (depends on robot)
    //This method drives straight with designated power, and all the while rotates to the designated heading
    public void DriveByEncoders(double heading, double power, int distance) {

        resetDistance();
        VecDrive(10, 0, power, 20000);
        yawPIDController.setSetPoint(heading);
        while (opmode.opModeIsActive()) {
            int d = getDistance();
            if (d > distance) break;
            BetterDarudeAutoNav.ADBLog("Dist: " + d);
            opmode.sleep(5);
        }
    }


    //Second movement method, uses NavX only to rotate to desired position
    public void TurnToAngle(double heading) {
        yawPIDController.setSetPoint(heading);

        try {
            while (opmode.opModeIsActive()) {
                //BetterDarudeAutoNav.ADBLog("TurnToAngle: Yaw=" + navx_device.getYaw() + ", AV=" + yawPIDController.getAV());
                if (Math.abs(navx_device.getYaw() - heading) < TOLERANCE_DEGREES
                        && Math.abs(yawPIDController.getAV()) < 0.01) break;
                sleep(20);
            }
        } catch (InterruptedException ex) {
            BetterDarudeAutoNav.ADBLog("Interrupted TurnToAngle");
        }
    }



    //A method used by several movement methods
    //maxDuration is the longest time that it can go, so it doesn't go forever if there is no new command
    public void VecDrive(double x, double y, double sp, int maxDuration) {

        //puts designated values
        x*=.5;
        //if values too small, sets it to 0 and  robot stops
        if ((Math.sqrt(Math.pow(x,2)+Math.pow(y,2))) < 0.05) {
            thrd.setReq(0,0,maxDuration);
        } else {
            thrd.setReq((x*(sp/Math.sqrt(Math.pow(x,2) + Math.pow(y,2))))/2, y*(sp/Math.sqrt(Math.pow(x,2)+Math.pow(y,2))),maxDuration);
        }
    }

    //Method used for correction of movement when voltage drops, increases magnitude proportional to drops.
    public double getVoltCoef() {
        double coef = 1;
        double volt = voltSensor.getVoltage();
        if (volt <= 9) {
            coef = 1.3;
        } else if (volt <= 10) {
            coef = 1.1;
        } else if (volt <= 11) {
            coef = 1.0;
        }

//        BetterDarudeAutoNav.ADBLog("Voltage: " + volt + ", coeff: " + coef);
        return coef;
    }

    //Alternate VecDrive, incorporates voltage correction. Basically does things to reqV to determine the needed movement vector
    public void VecDriveBalanced(double x, double y, double sp, int maxDuration) {
        synchronized (this) {
            sp *= getVoltCoef();


            if ((Math.sqrt(Math.pow(x,2)+Math.pow(y,2))) < 0.05) {
                thrd.setReq(0,0,maxDuration);
            } else {
                //here is the multiplication by 1/2 that we added but never worked, may need to remove
                thrd.setReq((x*(sp/Math.sqrt(Math.pow(x,2)+Math.pow(y,2))))/2,y*(sp/Math.sqrt(Math.pow(x,2)+Math.pow(y,2))),maxDuration);
            }
        }
    }
    public void brake() {
        thrd.brake();
    }

    public void Stop()
    {
        thrd.running = false;
        thrd.brake();
    }




    private int prevLF = 0;
    private int prevLB = 0;
    private int prevRF = 0;
    private int prevRB = 0;

    //Sets previous encoder values to current, makes a new reference point
    public void resetYDist() {
        prevLF = frontLeft.getCurrentPosition();
        prevLB = backLeft.getCurrentPosition();
        prevRF = frontRight.getCurrentPosition();
        prevRB = backRight.getCurrentPosition();
    }

    //Gets the increment of y distance. used for integrator  alongside time, to find distance travelled in y direction
    public double getYDistIncr() {

        int lf = frontLeft.getCurrentPosition();
        int lb = backLeft.getCurrentPosition();
        int left = (lf - prevLF) - (lb - prevLB);
        int rf = frontRight.getCurrentPosition();
        int rb = backRight.getCurrentPosition();
        int right = (rb - prevRB) - (rf - prevRF);

//        BetterDarudeAutoNav.ADBLog("lf: " + (lf-prevLF) + ", lb: " + (lb-prevLB) + ", rb: " + (rb-prevRB) + ", rf: " + (rf-prevRF));

        prevLF = lf;
        prevLB = lb;
        prevRF = rf;
        prevRB = rb;

        double res = (left + right);
        if (res < 0) return res / 16;
        else return res / 12;

    }

    //Method that is called from loop, sets motor power and uses pidcontroller to correct





//    public void brake_and_wait() {
//        try {
//            while (opmode.opModeIsActive()) {
//                synchronized (reqV) {
//                    delV.put(0, 0);
//                    delV.put(1, 0);
//                    reqV.put(0, 0);
//                    reqV.put(1, 0);
//                    powV.put(0, 0);
//                    powV.put(1, 0);
//                    lastCommandTime = runtime.milliseconds();
//                    driveDuration = 30000;
//                }
//                if (!navx_device.isMoving()) break;
////                BetterDarudeAutoNav.ADBLog("Waiting to stop");
//                sleep(30);
//            }
//        } catch (InterruptedException ex) {
//        }
//    }




    //Finds the distance travelled using encoders - important for DriveByEncoders
    public int getDistance() {
        int c = frontLeft.getCurrentPosition();
        int r = Math.abs(c - prevFLEnc);
        c = backLeft.getCurrentPosition();
        r += Math.abs(c - prevBLEnc);
        c = frontRight.getCurrentPosition();
        r += Math.abs(c - prevFREnc);
        c = backRight.getCurrentPosition();
        r += Math.abs(c - prevBREnc);

        BetterDarudeAutoNav.ADBLog("Distance: " + frontLeft.getCurrentPosition() + ", " + frontRight.getCurrentPosition() + ", " +
                backLeft.getCurrentPosition() + ", " + backRight.getCurrentPosition() + ", r:" + r);
        //ratio that reduces the size of values (encoders are in the 1000, this brings them to the 100)
        return r * 16 / 100;
    }

    //Resets distance, sets previous encoder value to current, making the current one be the new startpoint.
    public void resetDistance() {
        int c = frontLeft.getCurrentPosition();
        prevFLEnc = c;
        c = backLeft.getCurrentPosition();
        prevBLEnc = c;
        c = frontRight.getCurrentPosition();
        prevFREnc = c;
        c = backRight.getCurrentPosition();
        prevBREnc = c;
    }


    //Test methods
    public int testFrontLeft(double p) {
        frontLeft.setPower(p);
        return frontLeft.getCurrentPosition();
    }

    public int testBackLeft(double p) {
        backLeft.setPower(p);
        return backLeft.getCurrentPosition();
    }

    public int testFrontRight(double p) {
        frontRight.setPower(p);
        return frontRight.getCurrentPosition();
    }

    public int testBackRight(double p) {
        backRight.setPower(p);
        return backRight.getCurrentPosition();
    }

    //reverse if necessary
    public void ReverseDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    class internalThread implements Runnable {
        volatile double reqX = 0;
        volatile double driveDuration = 0;
        volatile double reqY = 0;
        volatile double lastCommandTime = 0;
        public volatile boolean running = true;

        public void Stop() {
            brake();
            running = false;
        }

        public void setReq(double x, double y, double dd)
        {
            synchronized (this) {
                reqX = x;
                reqY = y;
                driveDuration = dd;
                lastCommandTime = runtime.milliseconds();
                //BetterDarudeAutoNav.ADBLog("reqX: " + reqX + " reqY: "+ reqY + " driveDuration:" + driveDuration);
            }

        }

        public void run() {
            while (running) {

                double x,y, lct, dd;
                synchronized (this) {
                    x = reqX;
                    y = reqY;
                    lct = lastCommandTime;
                    dd = driveDuration;
                }

                //When time runs out, the vector is set back to zero.
                if (runtime.milliseconds() - lct > dd) {
                    // Stop
                    synchronized (this) {
                        reqX = 0;
                        reqY = 0;
                        lastCommandTime = runtime.milliseconds();
                        driveDuration = 30000;
                    }
//                    BetterDarudeAutoNav.ADBLog("Running too long. Stop!");
                }

//                BetterDarudeAutoNav.ADBLog("Running. Current pow: " + powV.get(0) + ":" + powV.get(1));
                //                BetterDarudeAutoNav.ADBLog("Running. Current req: " + reqV.get(0) + ":" + reqV.get(1));
//                BetterDarudeAutoNav.ADBLog("Running. Current del: " + delV.get(0) + ":" + delV.get(1));
                //sets movement angle based on vector components. also assigns motor power

                setMovement(x, y, 1);
            }

            brake();
            emergencyBrake();
        }

        //Sets the target vector to null, stopping the robot.
        public void brake() {
            synchronized (this) {
                reqY = 0;
                reqX = 0;
                lastCommandTime = runtime.milliseconds();
                driveDuration = 30000;
            }
        }

        //directly set motor values
        public void emergencyBrake() {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        //Method that is called from loop, sets motor power and uses pidcontroller to correct
        public void setMovement(double x, double y, double power) {
//        BetterDarudeAutoNav.ADBLog("Power: " + x + ", " + y);
            // Rotate 90 degrees

            //Variables for setting motor power. Scaled down to remain below 1.0
            double Xr = 0.707 * x + 0.707 * y;
            Xr *= power;
            double Yr = 0.707 * x - 0.707 * y;
            Yr *= power;
            boolean moving = (Math.abs(Xr) + Math.abs(Yr)) > 0.001;

            //This is used for rotating to the desired angle. note the details of PID controller that set it
            try {
                yawPIDController.setMoving(moving);
                if (yawPIDController.waitForNewUpdate(yawPIDResult, GYRO_DEVICE_TIMEOUT_MS)) {
                    double av = yawPIDResult.angular_velocity;
                    double error = yawPIDResult.error;
                    if (yawPIDResult.isOnTarget()) {
                        double flp = Xr;
                        double frp = Yr;
                        double blp = frp;
                        double brp = flp;
                        frontLeft.setPower(flp);
                        frontRight.setPower(frp);
                        backLeft.setPower(blp);
                        backRight.setPower(brp);
                        BetterDarudeAutoNav.ADBLog("On target motor speed: fl,br:" + flp + ", fr,bl: " + frp
                                + ", err: " + error + ", av= " + av);
                    } else {
                        double output = yawPIDResult.getOutput();
                        double flp = Xr;
                        double frp = Yr;
                        double blp = frp;
                        double brp = flp;

                        if (!moving) {
                            output = yawPIDResult.getStationaryOutput(0.1);
                        }

                        frontLeft.setPower(flp + output);
                        frontRight.setPower(frp - output);
                        backLeft.setPower(blp + output);
                        backRight.setPower(brp - output);
                        BetterDarudeAutoNav.ADBLog("Motor speed: fl,br:" + flp + ", fr,bl: " + frp
                                + ", err: " + error + ",out: " + output + ", av= " + av);
                    }
                    // Calculate odometer
//                double a = Math.toRadians(yawPIDController.prev_process_value);
//                int new_LCount = frontLeft.getCurrentPosition();
//                int new_RCount = frontRight.getCurrentPosition();
//                int average = ((prevLCount - new_LCount) + (prevRCount - new_RCount))/2;
//                deltaX += Math.cos(a)*average;
//                deltaY += Math.sin(a)*average;
//                prevLCount = new_LCount;
//                prevRCount = new_RCount;
//                BetterDarudeAutoNav.ADBLog("Odometer X: " + deltaX + ", Y:" + deltaY);
                } else {
                    /* A timeout occurred */
                    Log.d("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            } catch (Exception ex) {
            }
        }
    }

}


