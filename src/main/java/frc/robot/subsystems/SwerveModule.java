// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.Destination;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.DriveConstants.DriveMotor;
import frc.robot.util.Constants.DriveConstants.SteerMotor;
import frc.robot.util.Kraken;

public class SwerveModule extends SubsystemBase {
	private String canBusName;
	private CANcoder cancoder;
	private Kraken drive, steer;
	private int driveID, steerID, cancoderID;
	private double magnetOffset;

	private SwerveModuleState desiredState;

	public SwerveModule(String canBusName, int driveID, int steerID, int cancoderID, double magnetOffset) {
		this.canBusName = canBusName;
		this.driveID = driveID;
		this.steerID = driveID;
		this.cancoderID = driveID;
		this.magnetOffset = magnetOffset;

		desiredState = new SwerveModuleState();

		drive = new Kraken(driveID, canBusName);
		steer = new Kraken(steerID, canBusName);

		drive.setInverted(true);
		steer.setInverted(true);

		drive.setSupplyCurrentLimit(DriveConstants.kRriveMotorCurrentLimit);
		steer.setSupplyCurrentLimit(DriveConstants.kRriveMotorCurrentLimit);

		drive.setClosedLoopRampRate(0.1);
		steer.setClosedLoopRampRate(0.1);

		drive.setEncoder(0);
		steer.setEncoder(0);

		cancoder = new CANcoder(cancoderID, canBusName);

		CANcoderConfiguration config = new CANcoderConfiguration();
		// rotation from -0.5 to 0.5 instead of 0 to 1
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		config.MagnetSensor.MagnetOffset = -magnetOffset / (2 * Math.PI);
		cancoder.getConfigurator().apply(config);

		steer.setContinuousOutput();
		steer.setFeedbackDevice(cancoderID, FeedbackSensorSourceValue.FusedCANcoder);

		drive.setVelocityConversionFactor(DriveConstants.kRotorToDistanceRatio);
		steer.setRotorToSensorRatio(DriveConstants.kSteerMotorGearRatio);
		steer.setSensorToMechanismRatio(1);

		// drive.setVelocityConversionFactor(DriveConstants.);

		drive.setVelocityPIDValues(DriveMotor.kS, DriveMotor.kV, DriveMotor.kA, DriveMotor.kP,
				DriveMotor.kI, DriveMotor.kD, DriveMotor.kFF);
		steer.setVelocityPIDValues(SteerMotor.kS, SteerMotor.kV, SteerMotor.kA, SteerMotor.kP,
				SteerMotor.kI, SteerMotor.kD, SteerMotor.kFF, StaticFeedforwardSignValue.UseClosedLoopSign);
		//
	}

	double getCANCoderRadians() {
		// value as double in [-1/2, 1/2]
		// want in radian, multiple by 2pi
		return cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
	}

	public void setDesiredState(SwerveModuleState state) {
		// rotation2d wants radian
		state.optimize(new Rotation2d(getCANCoderRadians()));
		// state = SwerveModuleState.optimize(state, new
		// Rotation2d(getCANCoderRadians()));
		//
		desiredState = state;

		double optSteerAngle = state.angle.getRadians() / (2 * Math.PI);

		// setPosition already calculates the ratio that needs to be applied
		double rotorVelocity = state.speedMetersPerSecond;

		drive.setVelocityWithFeedForward(rotorVelocity);
		steer.setPositionWithFeedForward(optSteerAngle);
	}

	public SwerveModuleState getState() {
		// need wheel speed (in rotations) and rotation
		return new SwerveModuleState(drive.getMPS(), new Rotation2d(getCANCoderRadians()));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
				drive.getPosition() * DriveConstants.kRotorToDistanceRatio,
				new Rotation2d(getCANCoderRadians()));
	}

	public SwerveModuleState getDesiredState() {
		return desiredState;
	}

	public double getActualSpeed() {
		return drive.getMPS();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
