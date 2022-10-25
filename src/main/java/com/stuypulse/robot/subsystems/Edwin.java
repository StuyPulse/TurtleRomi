package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Ports.Edwin.*;
import static com.stuypulse.robot.constants.Settings.Edwin.*;
import static com.stuypulse.robot.constants.Motors.Edwin.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Edwin extends Robot {
    
    private final CANSparkMax[] left;
    private final CANSparkMax[] right;
    
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final Controller leftController, rightController;
    private SmartNumber leftTargetSpeed, rightTargetSpeed;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final AHRS navx;

    private final Field2d field;

    public Edwin() {
        left = new CANSparkMax[] {
            new CANSparkMax(LEFT_TOP, MotorType.kBrushless), 
            new CANSparkMax(LEFT_BOTTOM, MotorType.kBrushless)
        };

        right = new CANSparkMax[] {
            new CANSparkMax(RIGHT_TOP, MotorType.kBrushless), 
            new CANSparkMax(RIGHT_BOTTOM, MotorType.kBrushless)
        };

        LEFT_TOP_MOTOR.configure(left[0]);
        LEFT_BOTTOM_MOTOR.configure(left[1]);

        RIGHT_TOP_MOTOR.configure(right[0]);
        RIGHT_BOTTOM_MOTOR.configure(right[1]);

        leftEncoder = left[0].getEncoder();
        rightEncoder = right[0].getEncoder();

        leftEncoder.setVelocityConversionFactor(Encoders.GEAR_DISTANCE_PER_ROTATION / 60);
        rightEncoder.setVelocityConversionFactor(Encoders.GEAR_DISTANCE_PER_ROTATION / 60);
        
        leftController = new Feedforward.Drivetrain(Motion.Feedforward.kS, Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
            .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));
        
        rightController = new Feedforward.Drivetrain(Motion.Feedforward.kS, Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
            .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));

        leftTargetSpeed = new SmartNumber("Edwin/Left Target Speed", 0);
        rightTargetSpeed = new SmartNumber("Edwin/Right TargetSpeed", 0);

        odometry = new DifferentialDriveOdometry(getRotation2d());
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        navx = new AHRS(SPI.Port.kMXP);

        setPose(new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("Edwin/Field", field);
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    // Distance
    private double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    private double getRightDistance() {
        return rightEncoder.getPosition();
    }

    private double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    private double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    private double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /***************
     * ROBOT ANGLE *
     ***************/

    // Gets current Angle of the Robot as a Rotation2d (contiuous / not +-180)
    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    private void updateOdometry() {
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void setPose(Pose2d pose) {
        leftEncoder.setPosition(0.);
        rightEncoder.setPosition(0.);
        
        odometry.resetPosition(pose, getRotation2d());
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            Motion.MAX_VELOCITY, 
            Motion.MAX_ACCELERATION
        ).setKinematics(kinematics);
    }

    /********************
     * DRIVING COMMANDS *
     ********************/

    public void stop() {
        driveVolts(0., 0.);
    }

    public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
        leftTargetSpeed.set(leftMetersPerSecond);
        rightTargetSpeed.set(rightMetersPerSecond);
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        for (CANSparkMax motor : left) {
            motor.setVoltage(leftVolts);
        }

        for (CANSparkMax motor : right) {
            motor.setVoltage(rightVolts);
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());
        
        driveVolts(
            leftController.update(leftTargetSpeed.get(), getLeftVelocity()), 
            rightController.update(rightTargetSpeed.get(), getRightVelocity())
        );

        // LOGGING

        SmartDashboard.putNumber("Edwin/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Edwin/Right Distance", getRightDistance());
        SmartDashboard.putNumber("Edwin/Distance", getDistance());

        SmartDashboard.putNumber("Edwin/Left Vel", getLeftVelocity());
        SmartDashboard.putNumber("Edwin/Right Vel", getRightVelocity());
        SmartDashboard.putNumber("Edwin/Velocity", getVelocity());

        SmartDashboard.putNumber("Edwin/Pose X", getPose().getX());
        SmartDashboard.putNumber("Edwin/Pose Y", getPose().getY());
        SmartDashboard.putNumber("Edwin/Gyro Angle", getRotation2d().getDegrees());
    }

    @Override
    public Field2d getField2d() {
        return field;
    }
}
