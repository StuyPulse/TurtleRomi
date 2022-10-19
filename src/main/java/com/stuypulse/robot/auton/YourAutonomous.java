package com.stuypulse.robot.auton;
import com.stuypulse.robot.subsystems.Robot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class YourAutonomous extends SequentialCommandGroup {
    public YourAutonomous(Robot robot) {
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
