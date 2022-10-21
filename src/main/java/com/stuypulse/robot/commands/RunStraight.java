package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Robot;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunStraight extends CommandBase {
	
	private final Robot robot;
	private final StopWatch timer;

	public RunStraight(Robot robot) {
		this.robot = robot;
		timer = new StopWatch();

		addRequirements(robot);
	}

	@Override
	public void initialize() {
		timer.reset();
	}

	@Override
	public void execute() {
		robot.drive(0.5, 0.5);
	}

	@Override
	public boolean isFinished() {
		return timer.getTime() > 2.0;
	}

}
