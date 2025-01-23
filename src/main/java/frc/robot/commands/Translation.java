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
public class Translation extends Command {
  private Drivetrain drivetrain;
  private OI oi;
  private LimelightShooter limelightShooter;
  private double setpoint;
  private double currentTX;
  private double P, I, D;
  private PIDController pidController;
  private double translationalError;
  private double distance;

  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Translation(double setpoint) {
    this.setpoint=setpoint;
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();
    P = 0;
    I= 0;
    D= 0;
    pidController.setPID(P, I, D);

    SmartDashboard.putNumber("rotation P", P);
    SmartDashboard.putNumber("rotation I", I);
    SmartDashboard.putNumber("rotation D", D);

    translationalError = 0;
    currentTX = 0;
    distance = 0;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTX = limelightShooter.getTx();
    //distance = limelightShooter.getFilteredDistance();
    P = SmartDashboard.getNumber("rotation P", P);
    I = SmartDashboard.getNumber("rotation I", I);
    D = SmartDashboard.getNumber("rotation D", D);
    pidController.setPID(P, I, D);

    translationalError = currentTX - setpoint;

    drivetrain.drive(new Translation2d(pidController.calculate(translationalError, 0),0), 0, false, null);
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
