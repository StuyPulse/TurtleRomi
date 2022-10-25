// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot;

import static com.stuypulse.robot.Constants.Feedforward.*;

import com.stuypulse.robot.auton.*;
import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.keyboard.SimKeyGamepad;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link RobotLoop}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Robot robot = new SimRobot(kS, kV, kA);
  private final Gamepad gamepad = new SimKeyGamepad();
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    robot.setDefaultCommand(new RunCommand(() -> {
      robot.drive(gamepad.getLeftY(), gamepad.getRightY());
    }, robot));

    // Configure the button bindings
    configureButtonBindings();
    configureAutons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.
   * button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  private void configureAutons() {
    autonChooser.setDefaultOption("Your Autonomous", new YourAutonomous(robot));
    autonChooser.addOption("Straight", new RunStraight(robot));
    autonChooser.addOption("Straight Path", robot.fd(10));

    SmartDashboard.putData("Auton Chooser", autonChooser);
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
