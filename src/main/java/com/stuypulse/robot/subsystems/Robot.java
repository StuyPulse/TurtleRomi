package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.stuypulse.robot.commands.FollowPath;

public abstract class Robot extends SubsystemBase {

	public abstract void drive(double leftMetersPerSecond, double rightMetersPerSecond);
	public abstract Pose2d getPose();
	public abstract DifferentialDriveKinematics getKinematics();
	public abstract TrajectoryConfig getConstraints();

	public Command fd(double distance) {
		return new FollowPath(this,
			TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, new Rotation2d(0)),
				List.of(),
				new Pose2d(distance, 0, new Rotation2d(0)),
				getConstraints()));
	}

	public Command rt(double degrees) {

	}

}
