// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.LookupTable;

import java.rmi.dgc.DGC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AngleAlignment extends Command {
  private Drivetrain drivetrain;
  private LimelightShooter limelight;
  private PIDController controller;
  private double kP, kFF, threshold, error, setpoint, rotation;
  private double[] ids, angles;
  private LookupTable table;

  public AngleAlignment() {
    kP = 0.01;
    kFF = 0.01;
    threshold = 1;
    setpoint = 10.0; 
    error = 0;
    rotation = 0; 
    drivetrain = Drivetrain.getInstance();
    limelight = LimelightShooter.getInstance();
    controller = new PIDController(kP, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.

    SmartDashboard.putNumber("angle setpoint", 10.0); 
    SmartDashboard.putNumber("angle threshold", 1); 
    SmartDashboard.putNumber("angle kP", 0.01);
    SmartDashboard.putNumber("angle kFF", 0.01); 
    ids = new double[6]; 
    angles = new double[6]; 

    ids[0] = 6;
    angles[0] = 300;
    for(int i = 1; i<ids.length; i++){
      ids[i] = i+5;
      angles[i] = 60*i-60;
    }

    table = new LookupTable(ids, angles);

  }

  @Override
  public void initialize() {
    setpoint = table.get(limelight.getTargetID()); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kP = SmartDashboard.getNumber("angle kP", 0.01); 
    kFF = SmartDashboard.getNumber("angle kFF", 0.01); 

    threshold = SmartDashboard.getNumber("angle threshold", 1.0); 
    // setpoint = SmartDashboard.getNumber("angle setpoint", 10.0); 

    controller.setP(kP); 
    if(limelight.hasTarget()){
      error = setpoint - drivetrain.getHeading(); 
      if (error < -threshold){
        rotation = controller.calculate(setpoint) - kFF;
      } else if (error > threshold){
        rotation = controller.calculate(setpoint) + kFF;
      } else{
        rotation = 0;
      }
    }
    else{
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
