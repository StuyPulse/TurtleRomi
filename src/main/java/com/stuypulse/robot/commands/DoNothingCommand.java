package com.stuypulse.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingCommand extends CommandBase {
	
	public DoNothingCommand() {
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
