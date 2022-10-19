package com.stuypulse.robot.commands;

import com.stuypulse.robot.Constants.Constraints;
import com.stuypulse.robot.subsystems.Robot;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDelta extends CommandBase {

    private final Robot robot;
    private final Controller controller;

    private final double targetAngle;

    public TurnDelta(Robot robot, double deltaDegrees) {
        this.robot = robot;

        controller = new PIDController(1, 0, 0)
            .add(new Feedforward.Drivetrain(0.0, 1.0, 0.0).position()) // convert from angle position setpoint to velocity
            .setOutputFilter(new MotionProfile(Constraints.MAX_ANGULAR_VEL, Constraints.MAX_ANGULAR_VEL));
        
        targetAngle = robot.getGyroAngleDegrees() + deltaDegrees;

        addRequirements(robot);
    }

    @Override
    public void execute() {
        // controller might be outputting angles/s
        robot.turn(controller.update(targetAngle, robot.getGyroAngleDegrees()));
    }

    @Override
    public boolean isFinished() {
        return controller.isDone(0.5);
    }

}
