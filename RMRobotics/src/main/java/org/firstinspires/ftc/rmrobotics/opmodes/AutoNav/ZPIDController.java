package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Peter on 2/1/2017.
 */


public class ZPIDController implements IDataArrivalSubscriber {
    private final Object sync_event = new Object();
    private AHRS navx_device;
    private long last_sensor_timestamp = 0L;
    private long prev_sensor_timestamp = 0L;
    private double error_current = 0.0D;
    private double error_previous = 0.0D;
    private double p;

    private LinearOpMode logmode = null;
    private boolean debug = false;

    private double [][] StaticRules; // Rules while moving forward
    private double [][] MovingRules; // Rules while stationary
    private double [][] DerpRules; // Rules while moving in wrong direction

    private double max_input = 0.0D;
    private double min_input = 0.0D;
    private double max_output = 1.0D;
    private double min_output = -1.0D;
    private double tolerance_amount;
    private boolean enabled = false;
    private double setPoint = 0.0D;
    private double result = 0.0D;

    private boolean moving = false; // Is robot currently commanded to move
    private double angular_velocity = 0.0D;
    private double prev_process_value = 0;


    // Sets the zero angle to be the angle the robot is at right now
    public synchronized void yawReset() {
            this.last_sensor_timestamp = 0L;
            this.error_current = 0.0D;
            this.error_previous = 0.0D;
            this.result = 0.0D;
    }

    // Initialization
    public ZPIDController(AHRS navx_device, double[][] sr, double[][] mr, double[][] dr, boolean log) {
        this.navx_device = navx_device;
        this.setInputRange(-180.0D, 180.0D);
        navx_device.registerCallback(this);
        debug = log;
        StaticRules = sr;
        MovingRules = mr;
        DerpRules = dr;
    }

    public void close() {
        this.enable(false);
        this.navx_device.deregisterCallback(this);
    }

    // Block calling thread until new data available from sensor and control value is calculated.
    public synchronized boolean waitForNewUpdate(ZPIDController.PIDResult result, int timeout_ms) throws InterruptedException {
        // If new data already available, return it immediately
        boolean ready = isNewUpdateAvailable(result);

        // Exit loop when new data are available or thread is being interrupted.
        while(!ready && !Thread.currentThread().isInterrupted()) {
            // Yield lock to callback method
            wait((long)timeout_ms);
            ready = isNewUpdateAvailable(result);
        }

        return ready;
    }
    // Checking whether or not the PID controller has newer data, comparing to provided 'result'.
    // If yes, return true and store data into result.!!!!!
    // This function does not block the thread that calls it.
    // Called from the waitForNewUpdate method, which is a blocking method that waits for new update.
    public synchronized boolean isNewUpdateAvailable(ZPIDController.PIDResult result) {
        boolean new_data_available;
        if(enabled && result.timestamp < last_sensor_timestamp) {
            new_data_available = true;
            result.on_target = isOnTarget();
            result.output = get();
            result.timestamp = last_sensor_timestamp;
            result.angular_velocity = angular_velocity;
            result.error = error_current;
        } else {
            new_data_available = false;
        }

        return new_data_available;
    }

    // These two methods are callbacks from IDataArrivalSubscriber interface.
    // NavX thread calls them when new data is available from sensor.
    // We don't use untimestamped callback, since we expect navX micro to give timestamps
    public void untimestampedDataReceived(long curr_system_timestamp, Object kind) {}

    // curr_sensor_timestamp is a time when data was measured by sensor, not when it was received
    // by processing thread.
    // This method is running in navX thread and needs to be synchronized with drive thread.
    // We synchronize it with reading results from controller, to make sure no partial results
    // are read.
    public synchronized void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object kind) {
        if(enabled && kind.getClass() == AHRS.DeviceDataType.class) {
            double process_value = (double)navx_device.getYaw(); // controlled parameter

            prev_sensor_timestamp = last_sensor_timestamp;
            last_sensor_timestamp = curr_sensor_timestamp;

            // Calculate new control value
            stepController(process_value);

            // Notify drive thread that new control value is available
            notifyAll();
        }
    }

    //returns the differential between control and desired angle
    public double getError() {
        return error_current;
    }


    public synchronized void setTolerance(double tolerance_amount) {
        this.tolerance_amount = tolerance_amount;
    }

    //determines whether the robot is on target angle direction
    public boolean isOnTarget() {
        boolean on_target = false;
        on_target = Math.abs(this.getError()) < this.tolerance_amount;

        return on_target;
    }

    // Calculates control values. This method is called from navX callback and synchronized by its
    // lock.
    public double stepController(double process_variable) {


        //finds current error value. set point determined by setSetPoint, limited from -180 to 180, as is proccess variable (navx default)

        if (Math.abs(setPoint - process_variable) > 180)
        {
            if (setPoint - process_variable < 0) {
                error_current = setPoint - process_variable + 360;
            }
            else
            {
                error_current = setPoint - process_variable - 360;
            }
        }
        else
        {
            error_current = setPoint - process_variable;
        }

        //caclulates angular velocity
        angular_velocity = (process_variable - prev_process_value)/
                (last_sensor_timestamp - prev_sensor_timestamp);

        //absolute values for future use
        double absErr = Math.abs(error_current);
        double absAV = Math.abs(angular_velocity);



        double dump = 1;

        //determines dump value based on the array of values established

        if(angular_velocity == 0.0 || Math.signum(angular_velocity) == Math.signum(error_current)) {
            //if the robot is not moving in any particular direction, turning in place
            if (!moving) {
                int rn = 0;
                while (rn < StaticRules.length) {
                    if ((StaticRules[rn][1] < absAV) && StaticRules[rn][0] > absErr) {
                        dump = StaticRules[rn][2];
                        break;
                    }
                    rn++;
                }
            //if the robot is moving
            } else {
                int rn = 0;
                while (rn < MovingRules.length) {
                    if ((MovingRules[rn][1] < absAV) && MovingRules[rn][0] > absErr) {
                        dump = MovingRules[rn][2];
                        break;
                    }
                    rn++;
                }
            }
            //if the robot has overcorrected (angular velocity is opposite to the direction of error)
        } else {
            int rn = 0;
            while (rn < DerpRules.length) {
                if ((DerpRules[rn][1] < absAV) && DerpRules[rn][0] > absErr) {
                    dump = DerpRules[rn][2];
                    break;
                }
                rn++;
            }
        }


            // calculates result based on P value (modifiable), the resulting dump value and current error.
            result = p * error_current * dump;

            //sets previous values based on current ones
            error_previous = error_current;
            prev_process_value = process_variable;

        //corrects if output is out of bounds
            if(result > max_output) {
                result = max_output;
            } else if(result < min_output) {
                result = min_output;
            }

        if (debug)
        {
            BetterDarudeAutoNav.ADBLog("Angle: " + prev_process_value + ", AVel: " + angular_velocity + ", Res: " + result + ", error: " + (Math.signum(angular_velocity) == Math.signum(error_current)) + ", moving: " + (moving));

        }

            return result;
    }

    //sets the P value in "PID", only one we modify as it is the only one we use.
    public synchronized void setP(double p) {
        this.p = p;
//        this.stepController(this.error_previous, 0);
    }

    //extra method for getting result without recalculating
    public synchronized double get() {
        return result;
    }


    //establishes the output range for the PID controller, default is 1 to -1, however it is changed to -.5 to .5 in the Drive class (based on motor limitations)
    public synchronized void setOutputRange(double min_output, double max_output) {
        if(min_output <= max_output) {
            this.min_output = min_output;
            this.max_output = max_output;
        }

//        this.stepController(this.error_previous, 0);
    }


    public synchronized void setInputRange(double min_input, double max_input) {
        if(min_input <= max_input) {
            this.min_input = min_input;
            this.max_input = max_input;
            setSetPoint(setPoint);
        }
    }

    //converts setpoint into useable range (-180 to 180) and then assigns it to the class variable
    public synchronized void setSetPoint(double setPoint) {
        setPoint += 180;
        setPoint %= 360;
        setPoint -= 180;
        this.setPoint = setPoint;


        //to fix

//        this.stepController(this.error_previous, 0);
    }

    public synchronized double getSetPoint() {
        return this.setPoint;
    }

    public synchronized double getAV() { return angular_velocity; }
    public synchronized double getErr() { return error_current; }

    //if robot is not just rotating!
    public synchronized void setMoving(boolean m) { moving = m; }

    //method for resetting the controller
    public synchronized void enable(boolean enabled) {
        this.enabled = enabled;
        if(!enabled) {
            this.reset();
        }

    }

    public synchronized boolean isEnabled() {
        return this.enabled;
    }

    public synchronized void reset() {
        this.enabled = false;
        this.error_current = 0.0D;
        this.error_previous = 0.0D;
        this.result = 0.0D;
    }

//This is a container class that carries the values developped in the controller, it is released only when new data is found. (notifyAll must be called as the other thread is waiting)
    public static class PIDResult {
        public double output = 0.0D;
        public long timestamp = 0L;
        public boolean on_target = false;
        public double angular_velocity = 0;
        public double error = 0;

        public PIDResult() {
        }

        public long getTimestamp() {
            return this.timestamp;
        }

        public boolean isOnTarget() {
            return this.on_target;
        }

        public double getStationaryOutput(double min_stationary_output) {
                if (Math.abs(this.output) < min_stationary_output)
                    return Math.signum(this.output) * min_stationary_output;
                else return this.output;
        }
        public double getOutput() {
            return this.output;
        }
    }
}
