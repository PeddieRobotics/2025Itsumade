// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.HashMap;
import java.util.Map;


import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.Constants;
import frc.robot.util.LookupTable;

import java.rmi.dgc.DGC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GyroAlign extends Command {
  private Drivetrain drivetrain;
//   private LimelightShooter limelight;
  private PIDController controller;
  private double kP, kI, kD, kFF, threshold, error, setpoint, rotation, pidThreshold, thresholdP;

  // private double[] ids, angles;
  // private LookupTable table;

  public GyroAlign() {
    kP = 0.03;
    kI = 0.0;
    kD = 0.0;
    kFF = 0.0;
    threshold = .5;
    setpoint = 10.0; 
    error = 0;
    rotation = 0; 
    drivetrain = Drivetrain.getInstance();
    // limelight = LimelightShooter.getInstance();
    controller = new PIDController(kP, kI, kD);
    // Use addRequirements() here to declare subsystem dependencies.

    SmartDashboard.putNumber("angle setpoint", 10.0); 
    SmartDashboard.putNumber("angle threshold", 0.5); 
    SmartDashboard.putNumber("angle kP", 0.03);
    SmartDashboard.putNumber("angle kI", 0);
    SmartDashboard.putNumber("angle kD", 0);
    SmartDashboard.putNumber("angle kFF", 0.00); 
    SmartDashboard.putNumber("angle threshold P", 0.01); 
    SmartDashboard.putNumber("threshold P angle", 1.5); 
    

    pidThreshold = 1.5; 
    thresholdP = 0.01;
    
    

  }

  @Override
  public void initialize() {
    setpoint = 60; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kP = SmartDashboard.getNumber("angle kP", 0.03); 
    kI = SmartDashboard.getNumber("angle kI", 0.0); 
    kD = SmartDashboard.getNumber("angle kD", 0); 
    kFF = SmartDashboard.getNumber("angle kFF", 0.0); 

    thresholdP = SmartDashboard.getNumber("thresholdP angle", 1.5); 
    thresholdP = SmartDashboard.getNumber("angle threshold P", 0.05);
    threshold = SmartDashboard.getNumber("angle threshold", 0.5); 

    
    error = setpoint - drivetrain.getHeading(); 

    //if it is within p adjust threshold turn down the P 
    if(error > -pidThreshold && error < pidThreshold){
      controller.setP(thresholdP);
    }else{
      controller.setP(kP); 
    }
    controller.setI(kI); 
    controller.setD(kD); 


    //basic logic for PID 
    if (error < -threshold){
      rotation = controller.calculate(error) - kFF;
    } else if (error > threshold){
      rotation = controller.calculate(error) + kFF;
    }else{
      rotation = 0;
    }

    SmartDashboard.putNumber("angle error", error); 
    
    drivetrain.drive(new Translation2d(0,0), rotation, true, new Translation2d(0,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
