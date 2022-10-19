// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.Constants;
import com.stuypulse.robot.Constants.Constraints;
import com.stuypulse.robot.Constants.Ports;
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

public class Romi extends Robot {

  private final Spark leftMotor = new Spark(Ports.LEFT_MOTOR);
  private final Spark rightMotor = new Spark(Ports.RIGHT_MOTOR);

  private final Controller leftController, rightController;
  private double leftTargetSpeed, rightTargetSpeed;

  private final Encoder leftEncoder = new Encoder(Ports.LEFT_GRAYHILL_A, Ports.LEFT_GRAYHILL_B);
  private final Encoder rightEncoder = new Encoder(Ports.RIGHT_GRAYHILL_A, Ports.RIGHT_GRAYHILL_B);

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0));
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Encoder.TRACK_WIDTH_METERS);

  private final RomiGyro gyro = new RomiGyro();

  /** Creates a new RomiDrivetrain. */
  public Romi() {
    // Use meters as unit for encoder distances
    leftEncoder.setDistancePerPulse(Constants.Encoder.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.Encoder.DISTANCE_PER_PULSE);
    leftEncoder.reset();
    rightEncoder.reset();

    leftController = new Feedforward.Drivetrain(Constants.Feedforward.kS, Constants.Feedforward.kV, Constants.Feedforward.kA).velocity()
      .add(new PIDController(Constants.Feedback.kP, Constants.Feedback.kI, Constants.Feedback.kD));

    rightController = new Feedforward.Drivetrain(Constants.Feedforward.kS, Constants.Feedforward.kV, Constants.Feedforward.kA).velocity()
      .add(new PIDController(Constants.Feedback.kP, Constants.Feedback.kI, Constants.Feedback.kD));

    leftTargetSpeed = 0;
    rightTargetSpeed = 0;

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);
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

  @Override
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public TrajectoryConfig getTrajectoryConfig() {
    return new TrajectoryConfig(
        Constraints.MAX_VEL, 
        Constraints.MAX_ACC
    ).setKinematics(kinematics);
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    leftMotor.setVoltage(leftController.update(leftTargetSpeed, leftEncoder.getRate()));
    rightMotor.setVoltage(rightController.update(rightTargetSpeed, rightEncoder.getRate()));
  }
}
