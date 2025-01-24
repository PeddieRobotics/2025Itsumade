// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Logger;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WheelRadiusCharacterization extends Command {
 
  private Drivetrain drivetrain;
  private double currentGyro, initialGyro, deltaGyro;
  private double[] initialPositions, currentPositions;
  private double averageDeltaPositions;
  private double effectiveWheelRadius;
  private double drivebaseRadius;
  private Logger logger;

  /**
   * Creates a new wheelRadiusCharacterization
   .
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheelRadiusCharacterization() {
    drivetrain = Drivetrain.getInstance();
    logger = Logger.getInstance();
    currentGyro = 0;
    initialGyro = 0;
    deltaGyro = 0;
    initialPositions = new double[4];
    currentPositions = new double[4];
    averageDeltaPositions = 0;
    effectiveWheelRadius = 0;
    drivebaseRadius = Math.sqrt(Math.pow((DriveConstants.kTrackWidth/2),2) + Math.pow(DriveConstants.kWheelBase/2, 2)) ;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialGyro = drivetrain.getDegrees();
    initialPositions = Arrays.copyOf(drivetrain.getSwerveModuleRadianPosition(), 4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentGyro = drivetrain.getDegrees();
    deltaGyro = currentGyro - initialGyro;
    SmartDashboard.putNumber("deltaGyro", deltaGyro);

    drivetrain.drive(new Translation2d(0,0), 0.5, false, null);


    currentPositions = drivetrain.getSwerveModuleRadianPosition();
    SmartDashboard.putNumber("current module 0", currentPositions[0]);
    SmartDashboard.putNumber("current module 1", currentPositions[1]);
    SmartDashboard.putNumber("current module 2", currentPositions[2]);
    SmartDashboard.putNumber("current module 3", currentPositions[3]);
    SmartDashboard.putNumber("initial module 0", initialPositions[0]);
    SmartDashboard.putNumber("initla module 1", initialPositions[1]);
    SmartDashboard.putNumber("initla module 2", initialPositions[2]);
    SmartDashboard.putNumber("inital module 3", initialPositions[3]);

    for(int i = 0; i < 4; i++){
      averageDeltaPositions += (currentPositions[i] - initialPositions[i]);
    }
    averageDeltaPositions /= 4;

    effectiveWheelRadius = deltaGyro * drivebaseRadius / averageDeltaPositions;

    logger.deltaGyro.append(deltaGyro);
    logger.averageDeltaPositions.append(averageDeltaPositions);
    logger.effectiveWheelRadius.append(effectiveWheelRadius);
    SmartDashboard.putNumber("averageDeltaPositions", averageDeltaPositions);
    SmartDashboard.putNumber("drivebaseRadius", drivebaseRadius);
    SmartDashboard.putNumber("effectiveWheelRadius",effectiveWheelRadius);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("effectiveWheelRadius",effectiveWheelRadius);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
