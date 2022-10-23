package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class FollowPath extends RamseteCommand {

	private final Robot robot;
	private final Trajectory trajectory;

	private boolean robotRelative;

	public FollowPath(Robot robot, Trajectory trajectory) {
		super(
			trajectory,
			robot::getPose,
			new RamseteController(),
			robot.getKinematics(),
			robot::drive,
			robot
		);

		robotRelative = true;
		this.robot = robot;
		this.trajectory = trajectory;
	}

	@Override
	public void initialize() {
		super.initialize();

		if (robotRelative) {
			robot.reset(trajectory.getInitialPose());
		}
	}

	public FollowPath robotRelative() {
		robotRelative = true;
		return this;
	}

	public FollowPath fieldRelative() {
		robotRelative = false;
		return this;
	}

}
