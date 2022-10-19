// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public interface Ports {
		int LEFT_MOTOR = 0;
		int RIGHT_MOTOR = 1;
		int LEFT_GRAYHILL_A = 4;
		int LEFT_GRAYHILL_B = 5;
		int RIGHT_GRAYHILL_A = 6;
		int RIGHT_GRAYHILL_B = 7;
	}

	public interface Encoder {
		double COUNTS_PER_REVOLUTION = 1440.0;
		double WHEEL_DIAMETER_METERS = 0.07;
		double TRACK_WIDTH_METERS = 0.141;

		double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER_METERS) / COUNTS_PER_REVOLUTION;
	}

	public interface Feedforward {
		double kS = 0.0;
		double kV = 0.0;
		double kA = 0.0;
	}

	public interface Feedback {
		double kP = 1.0;
		double kI = 0.0;
		double kD = 0.0;
	}

	public interface Constraints {
		double MAX_VEL = 1.0;
		double MAX_ACC = 1.0;

		// rad/s^2
		double MAX_ANGULAR_VEL = (MAX_VEL * 2) / Constants.Encoder.TRACK_WIDTH_METERS;
		double MAX_ANGULAR_ACC = (Math.pow(MAX_ACC, 2)) / (Constants.Encoder.TRACK_WIDTH_METERS/2); // a = v^2 / r
	}
}