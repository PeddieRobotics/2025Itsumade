// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.Constants;
import frc.robot.util.OI;

import com.ctre.phoenix6.signals.PIDRefPIDErr_ClosedLoopModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Align extends Command {
  private Drivetrain drivetrain;
  private OI oi;
  private LimelightShooter limelightShooter;
  private double rotationalSetpoint;
  private double currentGyro, currentTX;
  private double rotationP, rotationI, rotationD;
  private double translationP, translationI, translationD;
  private PIDController rotationPidController, translationPidController;
  private double rotationalError;
  private double distance;
  private double ID;
  private double rotationalThreshold, txThreshold;
  private double lastSeenTx, lastSeenDistance;
  private int stage;

  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Align() {
    
    stage = 0;
    rotationalSetpoint = 0;
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();
    rotationP = 0;
    rotationI= 0;
    rotationD= 0;
    rotationPidController = new PIDController(rotationP, rotationI, rotationD);
   

  
    translationP = 0;
    translationI = 0;
    translationD = 0;
    translationPidController = new PIDController(translationP, translationI, translationD);

    SmartDashboard.putNumber("rotation P", rotationP);
    SmartDashboard.putNumber("rotation I", rotationI);
    SmartDashboard.putNumber("rotation D", rotationD);
    SmartDashboard.putNumber("translation P", translationP);
    SmartDashboard.putNumber("translation I", translationI);
    SmartDashboard.putNumber("translation D", translationD);

    rotationalError = 0;
    rotationalThreshold = 0.5;
    txThreshold = 1;
    currentGyro = 0;
    currentTX = 0;
    distance = 0;
    ID = 0;
    lastSeenTx = 0;
    lastSeenDistance = 0;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
    ID = limelightShooter.getTargetID();

    rotationalSetpoint =0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentGyro = drivetrain.getHeading();
    currentTX = limelightShooter.getTx();
    rotationP = SmartDashboard.getNumber("rotation P", rotationP);
    rotationI = SmartDashboard.getNumber("rotation I",rotationI);
    rotationD = SmartDashboard.getNumber("rotation D", rotationD);
    rotationPidController.setPID(rotationP, rotationI, rotationD);
    translationP = SmartDashboard.getNumber("translation P", translationP);
    translationI = SmartDashboard.getNumber("translation I",translationI);
    translationD = SmartDashboard.getNumber("translation D", translationD);
    translationPidController.setPID(translationP, translationI, translationD);
    

    if(limelightShooter.hasTarget()){
      lastSeenTx = currentTX;
      lastSeenDistance = limelightShooter.getFilteredDistance();
    }

    //rotation
    rotationalError = rotationalSetpoint - currentGyro;

    switch(stage){
      case 0:
        drivetrain.drive(new Translation2d(0,0), rotationPidController.calculate(rotationalError, 0), false, null);
        if(Math.abs(rotationalError) < rotationalThreshold){
          stage = 1;
        }
        break;
      
      case 1:
        drivetrain.drive(new Translation2d(0, -translationPidController.calculate(lastSeenTx, 0)), 0, false, null);
        if(Math.abs(lastSeenTx) < txThreshold){
          stage = 2;
        }
        break;

      case 2:
        drivetrain.drive(new Translation2d( -translationPidController.calculate(lastSeenDistance, 25), 0), 0, false, null);
        break;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(0,0), 0, false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
