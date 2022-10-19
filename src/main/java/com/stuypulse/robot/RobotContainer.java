// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.subsystems.Robot;
import com.stuypulse.robot.subsystems.Romi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link RobotLoop}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Robot robot = new Romi();

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureAutons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  private void configureAutons() {
    autonChooser.setDefaultOption("Romi Commands", new ControlRobot(robot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link RobotLoop} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
