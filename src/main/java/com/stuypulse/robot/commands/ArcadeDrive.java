package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Settings.Driver.*;
import static com.stuypulse.robot.constants.Settings.Edwin.Motion.*;

import com.stuypulse.robot.subsystems.Robot;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

    private final Robot robot;
    private final IStream speed, angle;

    public ArcadeDrive(Robot robot, Gamepad gamepad) {
        this.robot = robot;

        speed = IStream.create(() -> gamepad.getRightTrigger() - gamepad.getLeftTrigger()).filtered(
                                x -> SLMath.deadband(x, SPEED_DEADBAND.get()),
                                x -> SLMath.spow(x, SPEED_POWER.get()),
                                x -> x * MAX_VELOCITY,
                                new LowPassFilter(SPEED_FILTER));

        angle = IStream.create(() -> gamepad.getLeftX()).filtered(
                                x -> SLMath.deadband(x, ANGLE_DEADBAND.get()),
                                x -> SLMath.spow(x, ANGLE_POWER.get()),
                                x -> x * MAX_ANGULAR_VELOCITY,
                                new LowPassFilter(ANGLE_FILTER));

        addRequirements(robot);
    }

    public void execute() {
        robot.arcadeDrive(speed.get(), angle.get());
    }

    public boolean isFinished() {
        return false;
    }
}

