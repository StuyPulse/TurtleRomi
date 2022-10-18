package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class FollowPath extends RamseteCommand {

	public FollowPath(Robot robot, Trajectory trajectory) {
		super(
			trajectory.relativeTo(robot.getPose()),
			robot::getPose,
			new RamseteController(),
			robot.getKinematics(),
			robot::drive,
			robot
		);
	}

}
