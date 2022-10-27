package com.stuypulse.robot.auton;
import com.stuypulse.robot.subsystems.Robot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class YourAutonomous extends SequentialCommandGroup {
    public YourAutonomous(Robot robot) {
        addCommands(

            // Create a square

            robot.forward(4), 
            robot.right(90), 
            robot.back(4), 
            robot.right(90), 
            robot.forward(4), 
            robot.left(90),
            robot.forward(4),
            robot.left(90)
        );
    }
}
