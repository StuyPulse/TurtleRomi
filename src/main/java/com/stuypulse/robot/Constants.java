// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {
	public interface Ports {
		int LEFT_MOTOR = 0;
		int RIGHT_MOTOR = 1;

		int LEFT_ENCODER_A = 4;
		int LEFT_ENCODER_B = 5;

		int RIGHT_ENCODER_A = 6;
		int RIGHT_ENCODER_B = 7;

		public interface Edwin {
			int LEFT_TOP = 7;
            int LEFT_MIDDLE = 11;
            int LEFT_BOTTOM = 6;

            int RIGHT_TOP = 4;
            int RIGHT_MIDDLE = 14;
            int RIGHT_BOTTOM = 3;

			interface Encoders {
                int LEFT_A = 0;
                int LEFT_B = 1;

                int RIGHT_A = 2;
                int RIGHT_B = 3;
            }
		}
	}

	double TRACK_WIDTH_METERS = 0.141;
	double WHEEL_DIAMETER_METERS = 0.070;


	public interface Encoder {
		double COUNTS_PER_REVOLUTION = 1440.0;
		double WHEEL_DIAMETER_METERS = 0.07;
		
		double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER_METERS) / COUNTS_PER_REVOLUTION;
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
		double MAX_ANGULAR_VEL = (MAX_VEL * 2) / TRACK_WIDTH_METERS;
		double MAX_ANGULAR_ACC = (Math.pow(MAX_ACC, 2)) / (TRACK_WIDTH_METERS/2); // a = v^2 / r
	}

	public interface EdwinSettings {
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.4);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.05);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.05);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 1.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver Settings/Speed Filtering", 0.2);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.01);

        // Current Limit for the motors
        int CURRENT_LIMIT_AMPS = 60;

        // If the motors are inverted
        boolean IS_INVERTED = true;

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(30.0); // SEAN PROMISED !

        boolean USING_GRAYHILLS = false;
        boolean USING_GYRO = true;

        public interface Motion {

            double MAX_VELOCITY = 1.0;
            double MAX_ACCELERATION = 1.0;

            public interface Feedforward {
                double kS = 0.367; // TODO: characterize
                double kV = 2.07; // TODO: characterize
                double kA = 0.47; // TODO: characterize
            }

            public interface PID {
                double kP = 0.0198; // TODO: characterize
                double kI = 0; // TODO: characterize
                double kD = 0; // TODO: characterize
            }
        }

        public interface Odometry {
            Translation2d STARTING_TRANSLATION = new Translation2d();
            Rotation2d STARTING_ANGLE = new Rotation2d();

            Pose2d STARTING_POSITION = new Pose2d(STARTING_TRANSLATION, STARTING_ANGLE);
        }

        // Encoder Constants
        public interface Encoders {

            public interface GearRatio {
                double LOW_GEAR_NEO_TO_WHEEL =  (1.0 / 16.67); 

                double HIGH_GEAR_NEO_TO_WHEEL = (1.0 / 7.71);

                double GREYHILL_TO_WHEEL = 1.0;
            }

            double WHEEL_DIAMETER = Units.inchesToMeters(6);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double LOW_GEAR_DISTANCE_PER_ROTATION =
                    WHEEL_CIRCUMFERENCE * GearRatio.LOW_GEAR_NEO_TO_WHEEL;
            double HIGH_GEAR_DISTANCE_PER_ROTATION =
                    WHEEL_CIRCUMFERENCE * GearRatio.HIGH_GEAR_NEO_TO_WHEEL;

            double GREYHILL_PULSES_PER_REVOLUTION = 256;
            double GREYHILL_DISTANCE_PER_PULSE =
                    (WHEEL_CIRCUMFERENCE / GREYHILL_PULSES_PER_REVOLUTION)
                            * GearRatio.GREYHILL_TO_WHEEL;
        }
    }
}
