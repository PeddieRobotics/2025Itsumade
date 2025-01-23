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
public class AlignToReefAuto extends Command {
  private Drivetrain drivetrain;
  private OI oi;
  private LimelightShooter limelightShooter;

  private PIDController rotationPidController, translationPidController, distancePidController;
  private double rotationUseLowerPThreshold, rotationThresholdP;
  private double translationThreshold, rotationThreshold, distanceThreshold;
  private double desiredAngle;
  private double translationP, translationI, translationD, translationFF;
  private double rotationP, rotationI, rotationD, rotationFF;
  private double distanceP, distanceI, distanceD, distanceFF;
  private double desiredDistance;
  private double txValue, rotationError, distanceError;
  private double cycleCount;
  private Translation2d previousTranslation;
  
  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToReefAuto() {
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();

    desiredAngle = 0;
    desiredDistance = 17.0;

    SmartDashboard.putNumber("Desired Distance", desiredDistance);

    distanceP = 0.04;
    distanceI = 0;
    distanceD = 0.003;
    distanceFF = 0;
    distanceThreshold = 1;
    distancePidController = new PIDController(distanceP, distanceI , distanceD);

    SmartDashboard.putNumber("Distance P", distanceP);
    SmartDashboard.putNumber("Distance I", distanceI);
    SmartDashboard.putNumber("Distance D", distanceD);
    SmartDashboard.putNumber("Distance FF", distanceFF);
    
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
    SmartDashboard.putNumber("distanceThreshold", distanceThreshold);
    SmartDashboard.putNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

    previousTranslation = new Translation2d(0, 0);

    //constantSpeedAuto = 0.5;

    //SmartDashboard.putNumber("Constant Speed Auto", constantSpeedAuto);

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

    previousTranslation = new Translation2d(0, 0);

    cycleCount = 0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightShooter.setPipeline(drivetrain.getPipelineNumber());
    translationP = SmartDashboard.getNumber("PhilipAlign P", translationP);
    translationI = SmartDashboard.getNumber("PhilipAlign I", translationI);
    translationD = SmartDashboard.getNumber("PhilipAlign D", translationD);
    translationFF = SmartDashboard.getNumber("PhilipAlign FF", translationFF);

    rotationP = SmartDashboard.getNumber("Rotation P", rotationP);
    rotationThresholdP = SmartDashboard.getNumber("Rotation Threshold P", rotationThresholdP);
    rotationI = SmartDashboard.getNumber("Rotation I", rotationI);
    rotationD = SmartDashboard.getNumber("Rotation D", rotationD);
    rotationFF = SmartDashboard.getNumber("Rotation FF", rotationFF);

    distanceP = SmartDashboard.getNumber("Distance P", distanceP);
    distanceI = SmartDashboard.getNumber("Distance I", distanceI);
    distanceD = SmartDashboard.getNumber("Distance D", distanceD);
    distanceFF = SmartDashboard.getNumber("Distance FF", distanceFF);

    rotationThreshold = SmartDashboard.getNumber("rotationThreshold", rotationThreshold);
    translationThreshold = SmartDashboard.getNumber("translationThreshold", translationThreshold);
    distanceThreshold = SmartDashboard.getNumber("distanceThreshold", distanceThreshold);
    rotationUseLowerPThreshold = SmartDashboard.getNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

    desiredDistance = SmartDashboard.getNumber("Desired Distance", desiredDistance);

    rotationError = desiredAngle - drivetrain.getHeading();
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
    distancePidController.setPID(distanceP, distanceI, distanceD);

    // SmartDashboard.putNumber("PhilipAlign desired gyro angle", desiredAngle);
    // SmartDashboard.putNumber("PhilipAlign gyro angle", drivetrain.getHeading());
    // SmartDashboard.putNumber("PhilipAlign desired Tx", desiredTx);
    //
    // if (!limelightShooter.hasTarget() && lastTagSeen == -1) {
    //   drivetrain.drive(new Translation2d(0, 0), 0, false, null);
    //  return;
    // }

    double distance = limelightShooter.getFilteredDistance();

    
    double horizontalTranslation = 0;
    double forBackTranslation = 0;
    if (limelightShooter.hasTarget()) {
      txValue = limelightShooter.getTx();
      double translationError = 0;
      if (Math.abs(txValue) < 3) {
        translationError = txValue - desiredTx;
      } else {
        translationError = distance * Math.sin((txValue-desiredTx)*(Math.PI/180));
      }
      SmartDashboard.putNumber("Translation Error", translationError);
      distanceError = distance - desiredDistance;

      if (Math.abs(translationError) > translationThreshold)
        horizontalTranslation = translationPidController.calculate(translationError) - Math.signum(translationError) * translationFF;

      if (Math.abs(distanceError) > distanceThreshold)
        forBackTranslation = distancePidController.calculate(distanceError) - Math.signum(distanceError) * distanceFF;

      previousTranslation = new Translation2d(-forBackTranslation, -horizontalTranslation);
    }

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    // calculate forward-backward translation (a dot b)
    // double b1 = -oi.getStrafe();
    // double b2 = oi.getForward();
    // double forBackTranslation = (a1*b1 + a2*b2) * Constants.DriveConstants.kMaxFloorSpeed;

    //constantSpeedAuto = SmartDashboard.getNumber("Constant Speed Auto", constantSpeedAuto);

    drivetrain.drive(previousTranslation, rotation, false, null);

    cycleCount++;

    SmartDashboard.putBoolean("reef align is finished", isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("reef align is finished", true);
    drivetrain.drive(new Translation2d(0,0), 0, false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // |Tx| < .5 degree
    // |Gyro error| < 1 degreee
    // return false;
    return Math.abs(txValue) < 1 && Math.abs(rotationError) < 1 && cycleCount >= 10 && Math.abs(distanceError) < 1;
  }
}
