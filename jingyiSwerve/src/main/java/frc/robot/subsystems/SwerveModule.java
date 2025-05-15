// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
    private Kraken driveMotor, steerMotor;
    private CANcoder canCoder;

    private SwerveModuleState desiredState;

  public SwerveModule(String canbusName, int driveMotorID, int steerMotorID, int canCoderID, double moduleAngularOffset) {
    desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

    driveMotor = new Kraken(driveMotorID, canbusName);
    steerMotor = new Kraken(steerMotorID, canbusName);

    canCoder = new CANcoder(canCoderID, canbusName);

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setSupplyCurrentLimit(40.0);
    steerMotor.setSupplyCurrentLimit(40.0);

    driveMotor.setBrake();
    steerMotor.setCoast();

    driveMotor.setEncoder(0);
    steerMotor.setEncoder(0);

    driveMotor.setPIDValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    steerMotor.setPIDValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.MagnetOffset = moduleAngularOffset;

    canCoder.getConfigurator().apply(config);

    steerMotor.setContinuousOutput();
    steerMotor.setFeedbackDevice(canCoderID, FeedbackSensorSourceValue.RemoteCANcoder);

    driveMotor.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);
    
  }

  public double getCanCoderReading(){
    return canCoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
  }

  public SwerveModuleState getDesiredState(){
    return desiredState;
  }

  public void setDesiredState(SwerveModuleState currentDesiredState){
    desiredState = currentDesiredState;
    desiredState.optimize(new Rotation2d(getCanCoderReading()));

    double desiredAngle = desiredState.angle.getRadians()/(2.0*Math.PI);
    double desiredSpeed = desiredState.speedMetersPerSecond;

    steerMotor.setPositionVoltageWithFeedForward(desiredAngle);
    driveMotor.setPositionVoltageWithFeedForward(desiredSpeed);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    
  }
}

