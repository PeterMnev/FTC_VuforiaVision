package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;

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

    private double [][] Mrules; // Rules while moving forward
    private double [][] Srules; // Rules while stationary
    private double [][] Drules; // Rules while moving in wrong direction

    private double max_input = 0.0D;
    private double min_input = 0.0D;
    private double max_output = 1.0D;
    private double min_output = -1.0D;
    private double tolerance_amount;
    private boolean enabled = false;
    private double setpoint = 0.0D;
    private double result = 0.0D;

    private boolean moving = false; // Is robot currently commanded to move
    private double angular_velocity = 0.0D;
    private double prev_process_value = 0;

    // These two methods are callbacks from IDataArrivalSubscriber interface.
    // NavX thread calls them when new data are available from sensor.
    // We don't use untimestamped callback, since we expect navX micro to give timestamps/
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

    public synchronized void yawReset() {
            this.last_sensor_timestamp = 0L;
            this.error_current = 0.0D;
            this.error_previous = 0.0D;
            this.result = 0.0D;
    }

    public ZPIDController(AHRS navx_device, double[][] mr, double[][] sr, double[][] dr) {
        this.navx_device = navx_device;
        this.setInputRange(-180.0D, 180.0D);
        navx_device.registerCallback(this);
        Mrules = mr;
        Srules = sr;
        Drules = dr;
    }

    public void close() {
        this.enable(false);
        this.navx_device.deregisterCallback(this);
    }

    // Checking weather PID controller have newer data, comparing to provided 'result'.
    // If yes, return true and store data into result.
    // This function does not block thread that calls it.
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

    // Block calling thread until new data available from sensor and control value is calculated.
    public synchronized boolean waitForNewUpdate(ZPIDController.PIDResult result, int timeout_ms) throws InterruptedException {
        // If new data already available, return it immediately
        boolean ready = isNewUpdateAvailable(result);

        // Exit loop when new data are available or thread is being interrupted.
        if(!ready && !Thread.currentThread().isInterrupted()) {
            // Yield lock to callback method
            wait((long)timeout_ms);
            ready = isNewUpdateAvailable(result);
        }

        return ready;
    }

    public double getError() {
        return error_current;
    }

    public synchronized void setTolerance(double tolerance_amount) {
        this.tolerance_amount = tolerance_amount;
    }

    public boolean isOnTarget() {
        boolean on_target = false;
        on_target = Math.abs(this.getError()) < this.tolerance_amount;

        return on_target;
    }

    // Calculates control values. This method is called from navX callback and synchronized by its
    // lock.
    public double stepController(double process_variable) {
        error_current = setpoint - process_variable;
        double absErr = Math.abs(error_current);
        double absAV = Math.abs(angular_velocity);

        angular_velocity = (process_variable - prev_process_value)/
                    (last_sensor_timestamp - prev_sensor_timestamp);

        double dump = 1;

        if(angular_velocity == 0.0 || Math.signum(angular_velocity) == Math.signum(error_current)) {
            if (!moving) {
                int rn = 0;
                while (rn < Mrules.length) {
                    if ((Mrules[rn][1] < absAV) && Mrules[rn][0] > absErr) {
                        dump = Mrules[rn][2];
                        break;
                    }
                    rn++;
                }
            } else {
                int rn = 0;
                while (rn < Srules.length) {
                    if ((Srules[rn][1] < absAV) && Srules[rn][0] > absErr) {
                        dump = Srules[rn][2];
                        break;
                    }
                    rn++;
                }
            }
        } else {
            int rn = 0;
            while (rn < Drules.length) {
                if ((Drules[rn][1] < absAV) && Drules[rn][0] > absErr) {
                    dump = Drules[rn][2];
                    break;
                }
                rn++;
            }
        }

/*
            if(angular_velocity == 0.0 || Math.signum(angular_velocity) == Math.signum(error_current)) {
                // Turning in the right direction
                if(!moving) {
                    if (Math.abs(angular_velocity) > 0.11 && absErr < 30) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.09 && absErr < 18) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.075 && absErr < 15.5) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.06 && absErr < 13) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.01 && absErr < 5) {
                        dump = 0;  // stop just before target
                    }
                } else {
                    if (Math.abs(angular_velocity) > 0.06 && absErr < 20) {
                        dump = -10;  // stop rotation
                    } else if (Math.abs(angular_velocity) > 0.09 && absErr < 25) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.075 && absErr < 21) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.06 && absErr < 17) {
                        dump = 0; // brake
                    }
                }

//                else if(absErr < 20) adjP /=1.7;
            } else {
                // Turning in the wrong direction, apply maz correction
                if(Math.abs(angular_velocity) > 0.02) dump = 10;
            } */
//            BetterDarudeAutoNav.ADBLog("av: " + angular_velocity + ", err: " + absErr + ", dump: " + dump + ", mov: " + moving);

//            double inp_range;
//            if(continuous) {
//                inp_range = max_input - min_input;
//                if(absErr > inp_range / 2.0D) {
//                    if(error_current > 0.0D) {
//                        error_current -= inp_range;
//                    } else {
//                        error_current += inp_range;
//                    }
//                }
//            }

            result = p * error_current * dump;

            error_previous = error_current;

            if(result > max_output) {
                result = max_output;
            } else if(result < min_output) {
                result = min_output;
            }

            prev_process_value = process_variable;

            return result;
    }

    public synchronized void setP(double p) {
        this.p = p;
//        this.stepController(this.error_previous, 0);
    }

    public synchronized double get() {
        return result;
    }

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
            setSetpoint(setpoint);
        }
    }

    public synchronized void setSetpoint(double setpoint) {
        if(max_input > min_input) {
            if(setpoint > max_input) {
                this.setpoint = max_input;
            } else if(setpoint < min_input) {
                this.setpoint = min_input;
            } else {
                this.setpoint = setpoint;
            }
        } else {
            this.setpoint = setpoint;
        }

//        this.stepController(this.error_previous, 0);
    }

    public synchronized double getSetpoint() {
        return this.setpoint;
    }

    public synchronized double getAV() { return angular_velocity; }
    public synchronized double getErr() { return error_current; }

    public synchronized void setMoving(boolean m) { moving = m; }

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
