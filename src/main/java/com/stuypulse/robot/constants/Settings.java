package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

public interface Settings {

    public interface Driver {
        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.05);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.05);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 1.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver Settings/Speed Filtering", 0.2);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.01);
    }

    public interface Romi {

        public interface Robot {
            double TRACK_WIDTH_METERS = 0.141;
            double WHEEL_DIAMETER_METERS = 0.070;
        }
        
        public interface Encoder {
            double COUNTS_PER_REVOLUTION = 1440.0;
            
            double DISTANCE_PER_PULSE = (Math.PI * Robot.WHEEL_DIAMETER_METERS) / COUNTS_PER_REVOLUTION;
        }

        public interface Feedforward {
            double kS = 0.45;
            double kV = 10.00;
            double kA = 0.186;
        }

        public interface Feedback {
            double kP = 0.125;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface Constraints {
            double MAX_VEL = 1.0; // m/s
            double MAX_ACC = 1.0; // m/s^2

            // rad/s^2
            double MAX_ANGULAR_VEL = (MAX_VEL * 2) / Robot.TRACK_WIDTH_METERS;
            double MAX_ANGULAR_ACC = (Math.pow(MAX_ACC, 2)) / (Robot.TRACK_WIDTH_METERS/2); // a = v^2 / r
        }
    }

    public interface Edwin {

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(30.0); // SEAN PROMISED !

        public interface Motion {

            double MAX_VELOCITY = 1.0;
            double MAX_ACCELERATION = 1.0;

            double MAX_ANGULAR_VELOCITY = (MAX_VELOCITY * 2) / TRACK_WIDTH;
            double MAX_ANGULAR_ACCELERATION = (Math.pow(MAX_ACCELERATION, 2)) / (TRACK_WIDTH / 2);

            public interface Feedforward {
                double kS = 0.367; 
                double kV = 2.07; 
                double kA = 0.47; 
            }

            public interface PID {
                double kP = 0.00337; 
                double kI = 0; 
                double kD = 0; 
            }
        }

        public interface Encoders {
            
            double WHEEL_DIAMETER = Units.inchesToMeters(6);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            public interface GearRatio {
                // double LOW_GEAR_NEO_TO_WHEEL =  (1.0 / 16.67); 
                double HIGH_GEAR_NEO_TO_WHEEL = (1.0 / 7.71);
            }

            // double LOW_GEAR_DISTANCE_PER_ROTATION =
            //         WHEEL_CIRCUMFERENCE * GearRatio.LOW_GEAR_NEO_TO_WHEEL;
            double HIGH_GEAR_DISTANCE_PER_ROTATION =
                    WHEEL_CIRCUMFERENCE * GearRatio.HIGH_GEAR_NEO_TO_WHEEL;
        }
    }
}
