package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Robot;
import com.stuypulse.robot.util.RamseteCommand2;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class FollowPath extends RamseteCommand2 {

	private final Robot robot;
	private final Trajectory baseTrajectory;

	public FollowPath(Robot robot, Trajectory trajectory) {
		super(
			trajectory,
			robot::getPose,
			new RamseteController(),
			robot.getKinematics(),
			robot::drive,
			robot
		);

		this.robot = robot;
		this.baseTrajectory = trajectory;
	}

	@Override
	public void initialize() {
		m_trajectory = baseTrajectory.transformBy(new Transform2d(baseTrajectory.getInitialPose(), m_pose.get()));
		robot.getField2d().getObject("FollowPath/Trajectory").setTrajectory(m_trajectory);
		super.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		robot.getField2d().getObject("FollowPath/Trajectory").setTrajectory(new Trajectory());
		super.end(interrupted);
	}

}
