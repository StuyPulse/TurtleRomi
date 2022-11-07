// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Ports.Romi.*;
import static com.stuypulse.robot.constants.Settings.Romi.Robot.*;

import com.stuypulse.robot.util.Constraints;

import static com.stuypulse.robot.constants.Settings.Romi.Encoder.*;
import static com.stuypulse.robot.constants.Settings.Romi.Feedback.*;
import static com.stuypulse.robot.constants.Settings.Romi.Feedforward.*;
import static com.stuypulse.robot.constants.Settings.Romi.Constraints.*;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Romi extends Robot {

  private final Spark leftMotor = new Spark(LEFT_MOTOR);
  private final Spark rightMotor = new Spark(RIGHT_MOTOR);

  private final Controller leftController, rightController;
  private double leftTargetSpeed, rightTargetSpeed;


  private final Encoder leftEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
  private final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);

  private final DifferentialDriveOdometry odometry;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

  private final RomiGyro gyro = new RomiGyro();

  private final Constraints constraints;

  private final Field2d field = new Field2d();

  /** Creates a new RomiDrivetrain. */
  public Romi() {
    // Use meters as unit for encoder distances
    leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    leftEncoder.reset();
    rightEncoder.reset();

    // TODO: import statically
    leftController = new Feedforward.Drivetrain(kS, kV, kA).velocity()
      .add(new PIDController(kP, kI, kD));

    rightController = new Feedforward.Drivetrain(kS, kV, kA).velocity()
      .add(new PIDController(kP, kI, kD));

    leftTargetSpeed = 0;
    rightTargetSpeed = 0;

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);

    odometry = new DifferentialDriveOdometry(getRotation2d());

    constraints = new Constraints(TRACK_WIDTH_METERS, MAX_VEL, MAX_ACC, MAX_ANGULAR_VEL, MAX_ANGULAR_ACC, kinematics);

    SmartDashboard.putData(field);
  }

  public Constraints getConstraints() {
    return constraints;
  }

  @Override
  public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
    leftTargetSpeed = leftMetersPerSecond;
    rightTargetSpeed = rightMetersPerSecond;
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  @Override
  public void setPose(Pose2d pose) {
    resetEncoders();

    odometry.resetPosition(pose, new Rotation2d());
  }

  private double getRadians() {
    return (rightEncoder.getDistance() - leftEncoder.getDistance()) / TRACK_WIDTH_METERS;
  }

  @Override
  public Rotation2d getRotation2d() {
    // inverted to keep in robot convention of ccw positive
    return new Rotation2d(-getRadians());
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public TrajectoryConfig getTrajectoryConfig() {
    return new TrajectoryConfig(
        MAX_VEL, 
        MAX_ACC
    ).setKinematics(kinematics);
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(getPose());

    leftMotor.setVoltage(leftController.update(leftTargetSpeed, leftEncoder.getRate()));
    rightMotor.setVoltage(rightController.update(rightTargetSpeed, rightEncoder.getRate()));
  }

  @Override
  public Field2d getField2d() {
    return field;
  }
}
