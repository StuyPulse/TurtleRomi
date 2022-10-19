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

    private final double deltaRad;
    private double targetRad;

    public TurnDelta(Robot robot, double deltaRad) {
        this.robot = robot;

        // it's very funny how it's okay that this is not an angle controller (you don't have a continous system cause when delta > PI)
        controller = new PIDController(1, 0, 0)
            .add(new Feedforward.Drivetrain(0.0, 1.0, 0.0).position()) // convert from angle position setpoint to velocity
            .setSetpointFilter(new MotionProfile(Constraints.MAX_ANGULAR_VEL, Constraints.MAX_ANGULAR_ACC));
        
        this.deltaRad = deltaRad;

        addRequirements(robot);
    }

    @Override
    public void initialize() {
        targetRad = robot.getRotation2d().getRadians() + deltaRad;
    }

    @Override
    public void execute() {
        robot.turn(controller.update(targetRad, robot.getRotation2d().getRadians()));
    }

    @Override
    public boolean isFinished() {
        return controller.isDone(Math.toRadians(2.0));
    }

}
