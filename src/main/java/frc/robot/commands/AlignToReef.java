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
public class AlignToReef extends Command {
  private Drivetrain drivetrain;
  private OI oi;
  private LimelightShooter limelightShooter;

  private PIDController rotationPidController, translationPidController;
  private double rotationUseLowerPThreshold, rotationThresholdP;
  private double translationThreshold, rotationThreshold;
  private double desiredAngle;
  private double translationP, translationI, translationD, translationFF;
  private double rotationP, rotationI, rotationD, rotationFF;
  private double a1, a2;

  private int lastTagSeen;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToReef() {
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();

    desiredAngle = 0;

    translationP = 0.08;
    translationI = 0;
    translationD = 0;
    translationFF = 0;
    translationPidController = new PIDController(translationP, translationI , translationD);

    SmartDashboard.putNumber("PhilipAlign P", translationP);
    SmartDashboard.putNumber("PhilipAlign I", translationI);
    SmartDashboard.putNumber("PhilipAlign D", translationD);
    SmartDashboard.putNumber("PhilipAlign FF", translationFF);

    rotationP = 0.027;
    rotationI = 0.0;
    rotationD = 0.0;
    rotationFF = 0.0;
    rotationThresholdP = 0.01;
    rotationPidController = new PIDController(rotationP, rotationI, rotationD);

    SmartDashboard.putNumber("Rotation P", rotationP);
    SmartDashboard.putNumber("Rotation Threshold P", rotationThresholdP);
    SmartDashboard.putNumber("Rotation I", rotationI);
    SmartDashboard.putNumber("Rotation D", rotationD);
    SmartDashboard.putNumber("Rotation FF", rotationFF);

    rotationThreshold = 1;
    translationThreshold = 1;
    rotationUseLowerPThreshold = 1.5;
    
    SmartDashboard.putNumber("rotationThreshold", rotationThreshold);
    SmartDashboard.putNumber("translationThreshold", translationThreshold);
    SmartDashboard.putNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

    lastTagSeen = -1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();

    int desiredTarget = (int) limelightShooter.getTargetID();
    if (!Constants.kReefDesiredAngle.containsKey(desiredTarget))
      return;

    desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);
    a1 = Math.cos(Math.toRadians(desiredAngle));
    a2 = Math.sin(Math.toRadians(desiredAngle));

    lastTagSeen = desiredTarget;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translationP = SmartDashboard.getNumber("PhilipAlign P", translationP);
    translationI = SmartDashboard.getNumber("PhilipAlign I", translationI);
    translationD = SmartDashboard.getNumber("PhilipAlign D", translationD);
    translationFF = SmartDashboard.getNumber("PhilipAlign FF", translationFF);

    rotationP = SmartDashboard.getNumber("Rotation P", rotationP);
    rotationThresholdP = SmartDashboard.getNumber("Rotation Threshold P", rotationThresholdP);
    rotationI = SmartDashboard.getNumber("Rotation I", rotationI);
    rotationD = SmartDashboard.getNumber("Rotation D", rotationD);
    rotationFF = SmartDashboard.getNumber("Rotation FF", rotationFF);

    rotationThreshold = SmartDashboard.getNumber("rotationThreshold", rotationThreshold);
    translationThreshold = SmartDashboard.getNumber("translationThreshold", translationThreshold);
    rotationUseLowerPThreshold = SmartDashboard.getNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

    double rotationError = desiredAngle - drivetrain.getHeading();
    double desiredTx = -rotationError;

    // set rotation PID controller
    if(Math.abs(rotationError) < rotationUseLowerPThreshold)
      rotationPidController.setP(rotationThresholdP);
    else
      rotationPidController.setP(rotationP);
    rotationPidController.setI(rotationI);
    rotationPidController.setD(rotationD);

    // set translation PID controller
    translationPidController.setPID(translationP, translationI, translationD);

    // SmartDashboard.putNumber("PhilipAlign desired gyro angle", desiredAngle);
    // SmartDashboard.putNumber("PhilipAlign gyro angle", drivetrain.getHeading());
    // SmartDashboard.putNumber("PhilipAlign desired Tx", desiredTx);
    //
    // if (!limelightShooter.hasTarget() && lastTagSeen == -1) {
    //   drivetrain.drive(new Translation2d(0, 0), 0, false, null);
    //  return;
    // }

    double translation = 0;
    if (limelightShooter.hasTarget()) {
      double txValue = limelightShooter.getTx();
      double translationError = limelightShooter.getFilteredDistance() * Math.sin((txValue-desiredTx)*(Math.PI/180)) - 0;
      SmartDashboard.putNumber("Translation Error", translationError);

      if (Math.abs(translationError) > translationThreshold)
        translation = translationPidController.calculate(translationError) - Math.signum(translationError) * translationFF;
    }

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    // calculate forward-backward translation (a dot b)
    double b1 = -oi.getStrafe();
    double b2 = oi.getForward();
    double forBackTranslation = (a1*b1 + a2*b2) * Constants.DriveConstants.kMaxFloorSpeed;
    
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
