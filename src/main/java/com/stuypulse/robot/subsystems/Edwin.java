package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;

import static com.stuypulse.robot.Constants.*;
import static com.stuypulse.robot.Constants.EdwinSettings.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Edwin extends Robot {
    
    private final MotorControllerGroup left;
    private final MotorControllerGroup right;
    
    private final DifferentialDrive drivetrain;

    private final Encoder leftGreyhill;
    private final Encoder rightGreyhill;

    private final Controller leftController, rightController;
    private double leftTargetSpeed, rightTargetSpeed;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final AHRS navx;

    private final Field2d field;

    public Edwin() {
        left = new MotorControllerGroup(
            new CANSparkMax(Ports.Edwin.LEFT_TOP, MotorType.kBrushless), 
            new CANSparkMax(Ports.Edwin.LEFT_MIDDLE, MotorType.kBrushless),
            new CANSparkMax(Ports.Edwin.LEFT_BOTTOM, MotorType.kBrushless)
        );

        right = new MotorControllerGroup(
            new CANSparkMax(Ports.Edwin.RIGHT_TOP, MotorType.kBrushless), 
            new CANSparkMax(Ports.Edwin.RIGHT_MIDDLE, MotorType.kBrushless),
            new CANSparkMax(Ports.Edwin.RIGHT_BOTTOM, MotorType.kBrushless)
        );

        drivetrain = new DifferentialDrive(left, right);

        leftGreyhill = new Encoder(Ports.Edwin.Encoders.LEFT_A, Ports.Edwin.Encoders.LEFT_B);
        rightGreyhill = new Encoder(Ports.Edwin.Encoders.RIGHT_A, Ports.Edwin.Encoders.RIGHT_B);
        
        setGreyhillDistancePerPulse(EdwinSettings.Encoders.GREYHILL_DISTANCE_PER_PULSE);

        leftController = new Feedforward.Drivetrain(Motion.Feedforward.kS, Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
            .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));
        
        rightController = new Feedforward.Drivetrain(Motion.Feedforward.kS, Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
            .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));

        leftTargetSpeed = 0;
        rightTargetSpeed = 0;

        odometry = new DifferentialDriveOdometry(getRotation2d());
        kinematics = new DifferentialDriveKinematics(EdwinSettings.TRACK_WIDTH);
        navx = new AHRS(SPI.Port.kMXP);

        right.setInverted(true);
        reset(EdwinSettings.Odometry.STARTING_POSITION);

        field = new Field2d();
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    private void setGreyhillDistancePerPulse(double distance) {
        rightGreyhill.setDistancePerPulse(distance);
        rightGreyhill.reset();

        leftGreyhill.setDistancePerPulse(distance);
        leftGreyhill.reset();
    }

    // Distance
    private double getLeftDistance() {
        return leftGreyhill.getDistance();
    }

    private double getRightDistance() {
        return rightGreyhill.getDistance();
    }

    private double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    private double getLeftVelocity() {
        return leftGreyhill.getRate();
    }

    private double getRightVelocity() {
        return rightGreyhill.getRate();
    }

    private double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /***************
     * ROBOT ANGLE *
     ***************/

    // Gets current Angle of the Robot as a double (contiuous / not +-180)
    public double getRawGyroAngle() {
        return navx.getAngle();
    }

    // Gets current Angle of the Robot
    public Angle getGyroAngle() {
        return Angle.fromDegrees(getRawGyroAngle());
    }

    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    private void updateOdometry() {
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    }

    public Rotation2d getRotation2d() {
        // TODO: check if this needs to be negative
        return getGyroAngle().negative().getRotation2d();
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    @Override
    public void setPose(Pose2d pose) {
        reset();

        odometry.resetPosition(pose, new Rotation2d());
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            EdwinSettings.Motion.MAX_VELOCITY, 
            EdwinSettings.Motion.MAX_ACCELERATION
        ).setKinematics(kinematics);
    }

    /****************
     * SENSOR RESET *
     ****************/

    public void reset(Pose2d location) {
        leftGreyhill.reset();
        rightGreyhill.reset();

        odometry.resetPosition(location, getRotation2d());
    }

    public void reset() {
        reset(getPose());
    }

    /********************
     * DRIVING COMMANDS *
     ********************/

    public void stop() {
        drivetrain.stopMotor();
    }

    public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
        leftTargetSpeed = leftMetersPerSecond;
        rightTargetSpeed = rightMetersPerSecond;
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        left.setVoltage(leftController.update(leftTargetSpeed, getLeftVelocity()));
        right.setVoltage(rightController.update(rightTargetSpeed, getRightVelocity()));

        // LOGGING

        SmartDashboard.putNumber("Edwin/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Edwin/Right Distance", getRightDistance());
        SmartDashboard.putNumber("Edwin/Distance", getDistance());

        SmartDashboard.putNumber("Edwin/Left Vel", getLeftVelocity());
        SmartDashboard.putNumber("Edwin/Right Vel", getRightVelocity());
        SmartDashboard.putNumber("Edwin/Velocity", getVelocity());

        SmartDashboard.putNumber("Edwin/Pose X", getPose().getX());
        SmartDashboard.putNumber("Edwin/Pose Y", getPose().getY());
        SmartDashboard.putNumber("Edwin/Gyro Angle", getGyroAngle().toDegrees());
    }

    @Override
    public Field2d getField2d() {
        return field;
    }
}
