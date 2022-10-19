package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.stuypulse.robot.commands.FollowPath;
import com.stuypulse.robot.commands.TurnDelta;

public abstract class Robot extends SubsystemBase {

	/** KINEMATICS & ODOMETRY */

	public abstract Pose2d getPose();
	public abstract Rotation2d getRotation2d();
	public abstract DifferentialDriveKinematics getKinematics();

	protected abstract TrajectoryConfig getTrajectoryConfig();

	/** TELEOP CONTROL **/

	public abstract void drive(double leftMetersPerSecond, double rightMetersPerSecond);
	
	public final void turn(double omega) {
		var wheelSpeeds = getKinematics().toWheelSpeeds(new ChassisSpeeds(0, 0, omega));
		drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
	}

	/** AUTONOMOUS CONTROL */

	public final Command fd(double distance) {
		return new FollowPath(this,
			TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, new Rotation2d(0)),
				List.of(),
				new Pose2d(Units.inchesToMeters(distance), 0, new Rotation2d(0)),
				getTrajectoryConfig()));
	}

	public final Command rt(double degrees) {
		return new TurnDelta(this, Math.toRadians(-degrees));
	}

	public final Command lt(double degrees) {
		return new TurnDelta(this, Math.toRadians(+degrees));
	}

}
