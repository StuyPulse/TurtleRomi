package com.stuypulse.robot;
import com.stuypulse.robot.subsystems.Robot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ControlRobot extends SequentialCommandGroup {
    public ControlRobot(Robot robot) {
        addCommands(

            // Create a square

            robot.fd(10), 
            robot.rt(90), 
            robot.fd(10), 
            robot.rt(90), 
            robot.fd(10), 
            robot.rt(90)
        );
    }
}
