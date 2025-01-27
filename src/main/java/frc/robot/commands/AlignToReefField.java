// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.util.Constants;
import frc.robot.util.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReefField extends Command {
  private Drivetrain drivetrain;
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
  private double startTime;
  private Translation2d asdfadsfadfTranslation;
  private boolean isAuto;
  
  private Translation2d forBackUnitVec;
  private Translation2d horizontalUnitVec;

  private Logger logger;
  
  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToReefField(boolean isAuto) {
    this.isAuto = isAuto;

    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();
    logger = Logger.getInstance();

    desiredAngle = 0;
    desiredDistance = 17.0;

    SmartDashboard.putNumber("Desired Distance", desiredDistance);

    distanceP = 0.034;
    distanceI = 0;
    distanceD = 0.003;
    distanceFF = 0;
    distanceThreshold = 1;
    distancePidController = new PIDController(distanceP, distanceI , distanceD);

    SmartDashboard.putNumber("Distance P", distanceP);
    SmartDashboard.putNumber("Distance I", distanceI);
    SmartDashboard.putNumber("Distance D", distanceD);
    SmartDashboard.putNumber("Distance FF", distanceFF);
    
    translationP = 0.05;
    translationI = 0;
    translationD = 0;
    translationFF = 0.001;
    translationPidController = new PIDController(translationP, translationI , translationD);

    SmartDashboard.putNumber("PhilipAlign P", translationP);
    SmartDashboard.putNumber("PhilipAlign I", translationI);
    SmartDashboard.putNumber("PhilipAlign D", translationD);
    SmartDashboard.putNumber("PhilipAlign FF", translationFF);

    rotationP = 0.07;
    rotationI = 0.0;
    rotationD = 0.0;
    rotationFF = 0.0;
    rotationThresholdP = 0.04;
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

    asdfadsfadfTranslation = new Translation2d(0, 0);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int desiredTarget = (int) limelightShooter.getTargetID();
    if (!Constants.kReefDesiredAngle.containsKey(desiredTarget))
      return;

    desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);
    
    // t = desiredAngle; facing t = directly towards the wall
    // therefore forward and back would have angle t
    double theta = Math.toRadians(desiredAngle);
    forBackUnitVec = new Translation2d(
        Math.cos(theta), Math.sin(theta)
    );
    
    // so the translation would be (cos(t+90), sin(t+90))
    // cos(t+90) = cos(t)cos(90)-sin(t)sin(90) = -sin(t)
    // sin(t+90) = sin(t)cos(90)+cos(t)sin(90) = cos(t)
    // notice the two vectors are indeed orthogonal
    horizontalUnitVec = new Translation2d(
        -Math.sin(theta), Math.cos(theta)
    );

    asdfadsfadfTranslation = new Translation2d(0, 0);
    startTime = Timer.getFPGATimestamp();
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


    // TODO: see if translation is + or -
    if (limelightShooter.hasTarget()) {
      double distance = limelightShooter.getFilteredDistance();
      txValue = limelightShooter.getTx();

      double translationError = 0;
      if (Math.abs(txValue) < 3)
        translationError = txValue - desiredTx;
      else
        translationError = distance * Math.sin((txValue-desiredTx)*(Math.PI/180));

      SmartDashboard.putNumber("Translation Error", translationError);
      SmartDashboard.putNumber("Distance Error", distanceError);
      distanceError = distance - desiredDistance;

      double horizontalTranslation = 0, forBackTranslation = 0;
      if (Math.abs(translationError) > translationThreshold)
        horizontalTranslation = translationPidController.calculate(translationError) - Math.signum(translationError) * translationFF;

      if (Math.abs(distanceError) > distanceThreshold)
        forBackTranslation = distancePidController.calculate(distanceError) - Math.signum(distanceError) * distanceFF;

      Translation2d horizontalVector = horizontalUnitVec.times(-horizontalTranslation);
      Translation2d forBackVector = forBackUnitVec.times(-forBackTranslation);
      asdfadsfadfTranslation = horizontalVector.plus(forBackVector);

      logger.cmdTranslationEntry.append(translationError);
      logger.cmdDistanceEntry.append(distanceError);
    }
    // else
    //   translation = new Translation2d(translation.getX() / 2, translation.getY() / 2);

    SmartDashboard.putNumber("Rotation Error", rotationError);
    logger.cmdRotationEntry.append(rotationError);

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    logger.cmdCommandXEntry.append(asdfadsfadfTranslation.getX());
    logger.cmdCommandYEntry.append(asdfadsfadfTranslation.getY());
    logger.cmdCommandRotationEntry.append(rotation);

    drivetrain.drive(asdfadsfadfTranslation, rotation, true, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double elapsedTime = Timer.getFPGATimestamp() - SmartDashboard.getNumber("Forward start", startTime);
    SmartDashboard.putNumber("time elapsed since start", elapsedTime);

    drivetrain.drive(new Translation2d(0,0), 0, true, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // |Tx| < .5 degree
    // |Gyro error| < 1 degreee
    // return false;
    if (!isAuto)
      return false;
    double elapsed = Timer.getFPGATimestamp() - startTime;
    if (elapsed >= 20) {
      SmartDashboard.putBoolean("ended by time", true);
      return true;
    }
    SmartDashboard.putBoolean("ended by time", false);
    return Math.abs(txValue) < 1 && Math.abs(rotationError) < 1000 && Math.abs(distanceError) < 1 && elapsed >= 0.1;
  }
}
