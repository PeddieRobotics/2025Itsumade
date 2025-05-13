// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class SwerveModule extends SubsystemBase {

  private Kraken driveMotor, steerMotor;
  private CANcoder cancoder;

  private SwerveModuleState desiredState;

  public SwerveModule(String canbusName, int driveMotorID, int steerMotorID, int canCoderID, double moduleAngularOffset){
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    driveMotor = new Kraken(driveMotorID, canbusName);
    steerMotor = new Kraken(steerMotorID, canbusName);
    cancoder = new CANcoder(canCoderID, canbusName);

    //so it moves the same as if it weren't flipped
    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setSupplyCurrentLimit(40);
    steerMotor.setSupplyCurrentLimit(40);

    driveMotor.setBrake();
    steerMotor.setCoast();

    driveMotor.setEncoder(0);
    steerMotor.setEncoder(0);

    driveMotor.setPIDValues(0, 0, 0, 0, 0, 0, 0);
    steerMotor.setPIDValues(0, 0, 0, 0, 0, 0, 0);

    //where it wraps [-0.5,0.5]
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cancoderConfig.MagnetSensor.MagnetOffset = moduleAngularOffset;

    cancoder.getConfigurator().apply(cancoderConfig);

    //double velocity + rotation2D angle
    desiredState = new SwerveModuleState(0,new Rotation2d(0));
    
  }

  //absolute position does not reset position to 0 everytime you restart
  //convert from rotations to radians
  public double getCancoderReading(){
    return cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  public SwerveModuleState getDesiredState(){
    return desiredState;
  }

  public void setDesiredState(SwerveModuleState currentDesiredState){
    desiredState = currentDesiredState;
    //convert cancoderreading to rotation2d bc optimize only takes that
    desiredState.optimize(new Rotation2d(getCancoderReading()));

    double desiredAngle = desiredState.angle.getRotations();
    double desiredVelocity = desiredState.speedMetersPerSecond;

    steerMotor.setPositionVoltageWithFeedForward(desiredAngle);
    driveMotor.setVelocityVoltageWithFeedForward(desiredVelocity);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
