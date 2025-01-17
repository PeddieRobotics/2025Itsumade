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
public class PhilipAlignmentCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Drivetrain drivetrain;
  private OI oi;
  private LimelightShooter limelightShooter;
  private PIDController pidController;
  private double threshold;
  private double P, I, D, FF;
  private double a1, a2;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PhilipAlignmentCommand() {
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();

    P = 0.09;
    I = 0;
    D = 0;
    FF = 0;
    pidController = new PIDController(P, I , D);

    SmartDashboard.putNumber("PhilipAlign P", P);
    SmartDashboard.putNumber("PhilipAlign I", I);
    SmartDashboard.putNumber("PhilipAlign D", D);
    SmartDashboard.putNumber("PhilipAlign FF", FF);
    
    threshold = 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();

    int desiredTarget = (int) limelightShooter.getTargetID();
    double desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);
    a1 = Math.cos(Math.toRadians(desiredAngle));
    a2 = Math.sin(Math.toRadians(desiredAngle));
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double txValue = limelightShooter.getTx();
    double error = limelightShooter.getFilteredDistance() * Math.sin(txValue*(Math.PI/180)) - 4;

    SmartDashboard.putNumber("PhilipAlign Error", error);

    if (!limelightShooter.hasTarget()){
     return;
    }

    P = SmartDashboard.getNumber("PhilipAlign P", P);
    I = SmartDashboard.getNumber("PhilipAlign I", I);
    D = SmartDashboard.getNumber("PhilipAlign D", D);
    FF = SmartDashboard.getNumber("PhilipAlign FF", FF);

    pidController.setPID(P, I, D);

    double translation = 0;


    if(Math.abs(error) > threshold){
      translation = pidController.calculate(error) - Math.signum(error) * FF;
    }

    double rotation = oi.getRotation();
    
    double b1 = oi.getStrafe();
    double b2 = oi.getForward();

    // a = (a1, a2) b = (b1, b2)
    double forBackTranslation = a1*b1 + a2*b2;
    
    drivetrain.drive(new Translation2d(forBackTranslation, -translation), rotation, false, null);

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
