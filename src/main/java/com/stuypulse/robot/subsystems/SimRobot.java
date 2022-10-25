package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.Constants.Feedback.*;
import com.stuypulse.robot.Constants;
import com.stuypulse.robot.Constants.Constraints;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimRomi extends Robot {

	private final DifferentialDrivetrainSim sim;

	private final Controller leftController, rightController;
	private SmartNumber leftTargetSpeed, rightTargetSpeed;


	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
	private final DifferentialDriveOdometry odometry;

	private final Field2d field;

	public SimRomi(double kS, double kV, double kA) {
		sim = new DifferentialDrivetrainSim(
			LinearSystemId.identifyDrivetrainSystem(
				kV, kA, 10, 5),
			DCMotor.getRomiBuiltIn(2),
			1.0,
			Constants.TRACK_WIDTH_METERS,
			Constants.WHEEL_DIAMETER_METERS / 2,
			new Matrix<N7, N1>(Nat.N7(), Nat.N1()));

		// TODO: import statically
		leftController = new Feedforward.Drivetrain(kS, kV, kA).velocity()
		   .add(new PIDController(kP, kI, kD));
		rightController = new Feedforward.Drivetrain(kS, kV, kA).velocity()
		   .add(new PIDController(kP, kI, kD));

		leftTargetSpeed = new SmartNumber("Target Left Vel", 0);
		rightTargetSpeed = new SmartNumber("Target Right Vel", 0);

		odometry = new DifferentialDriveOdometry(getRotation2d());

		field = new Field2d();
		SmartDashboard.putData(field);
	}

	public Field2d getField2d() {
		return field;
	}

	@Override
	public void drive(double leftMetersPerSecond, double rightMetersPerSecond) {
		leftTargetSpeed.set(leftMetersPerSecond);
		rightTargetSpeed.set(rightMetersPerSecond);
	}

	@Override
	public Pose2d getPose() {
		return sim.getPose();
	}

	@Override
	public void setPose(Pose2d pose) {
		sim.setPose(pose);

		odometry.resetPosition(pose, new Rotation2d());
	}

	@Override
	public Rotation2d getRotation2d() {
		return sim.getHeading();
	}

	@Override
	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}

	@Override
	protected TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(
       		Constraints.MAX_VEL,
        	Constraints.MAX_ACC
    	).setKinematics(kinematics);
	}

	private static double clamp(double x) {
		return MathUtil.clamp(x, -RoboRioSim.getVInVoltage(), +RoboRioSim.getVInVoltage());
	}

	@Override
	public void periodic() {
	  odometry.update(getRotation2d(), sim.getLeftPositionMeters(), sim.getRightPositionMeters());

		sim.setInputs(
			clamp(leftController.update(leftTargetSpeed.get(), sim.getLeftVelocityMetersPerSecond())),
			clamp(rightController.update(rightTargetSpeed.get(), sim.getRightVelocityMetersPerSecond()))
		);

		sim.update(0.02);

		field.setRobotPose(getPose());

		SmartDashboard.putNumber("Left Voltage", clamp(leftController.getOutput()));
		SmartDashboard.putNumber("Right Voltage", clamp(rightController.getOutput()));
		SmartDashboard.putNumber("Left Velocity", sim.getLeftVelocityMetersPerSecond());
		SmartDashboard.putNumber("Right Velocity", sim.getRightVelocityMetersPerSecond());
	}

}
