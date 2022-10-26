package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Settings.Romi.Robot.*;

import com.stuypulse.robot.util.Constraints;

import static com.stuypulse.robot.constants.Settings.Romi.Constraints.*;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PerfectSimRomi extends Robot {

    private final SmartNumber leftTargetSpeed, rightTargetSpeed;
    private double leftDistance, rightDistance;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    private final Constraints constraints;

    private final Field2d field;

    public PerfectSimRomi() {
        leftTargetSpeed = new SmartNumber("Target Left Vel", 0);
		rightTargetSpeed = new SmartNumber("Target Right Vel", 0);

        leftDistance = 0.0;
        rightDistance = 0.0;

        odometry = new DifferentialDriveOdometry(new Rotation2d());
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        constraints = new Constraints(TRACK_WIDTH_METERS, MAX_VEL, MAX_ACC, MAX_ANGULAR_VEL, MAX_ANGULAR_ACC, kinematics);

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public Constraints getConstraints() {
        return constraints;
    }

    @Override
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void setPose(Pose2d pose) {
        leftDistance = 0.0;
        rightDistance = 0.0;
     
        odometry.resetPosition(pose, getRotation2d());
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d((rightDistance - leftDistance)/TRACK_WIDTH_METERS);
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    protected TrajectoryConfig getTrajectoryConfig() {
        return new TrajectoryConfig(
            MAX_VEL, 
            MAX_ACC
        ).setKinematics(kinematics);
    }

    @Override
    public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
        leftTargetSpeed.set(leftMetersPerSecond);
        rightTargetSpeed.set(rightMetersPerSecond);
    }

    @Override
    public void periodic() {
        leftDistance += leftTargetSpeed.get() * 0.02;
        rightDistance += rightTargetSpeed.get() * 0.02;

        odometry.update(getRotation2d(), leftDistance, rightDistance);

        field.setRobotPose(getPose());
    }

    @Override
    public Field2d getField2d() {
        return field;
    }
    
}
