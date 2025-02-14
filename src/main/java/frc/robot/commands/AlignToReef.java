// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightPVShooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.Constants;
import frc.robot.util.OI;

import frc.robot.util.Logger;

import com.ctre.phoenix6.signals.PIDRefPIDErr_ClosedLoopModeValue;
import com.fasterxml.jackson.databind.DeserializationFeature;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReef extends Command {
  private Drivetrain drivetrain;
  private OI oi;
  private LimelightPVShooter shooterCam;

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
  private double desiredTranslation;
  private Translation2d translation;
  private boolean isAuto;

  private Logger logger;
  
  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToReef(boolean isAuto) {
    this.isAuto = isAuto;

    drivetrain = Drivetrain.getInstance();
    shooterCam = LimelightPVShooter.getInstance();
    logger = Logger.getInstance();

    desiredAngle = 0;
    desiredDistance = 0.72;
    desiredTranslation = 0.07;

    SmartDashboard.putNumber("Desired Distance", desiredDistance);
    SmartDashboard.putNumber("Desired Translation", desiredTranslation);

    distanceP = 2;
    distanceI = 0;
    distanceD = 0;
    // distanceD = 0.0762;
    distanceFF = 0;
    distancePidController = new PIDController(distanceP, distanceI , distanceD);

    SmartDashboard.putNumber("Distance P", distanceP);
    SmartDashboard.putNumber("Distance I", distanceI);
    SmartDashboard.putNumber("Distance D", distanceD);
    SmartDashboard.putNumber("Distance FF", distanceFF);
    
    translationP = 1.75;
    translationI = 0;
    translationD = 0;
    translationFF = 0.001;
    translationPidController = new PIDController(translationP, translationI , translationD);

    SmartDashboard.putNumber("PhilipAlign P", translationP);
    SmartDashboard.putNumber("PhilipAlign I", translationI);
    SmartDashboard.putNumber("PhilipAlign D", translationD);
    SmartDashboard.putNumber("PhilipAlign FF", translationFF);

    rotationP = 0.05;
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

    distanceThreshold = 0.0254;
    rotationThreshold = 1;
    translationThreshold = 0.0254;
    rotationUseLowerPThreshold = 1.5;
    
    SmartDashboard.putNumber("rotationThreshold", rotationThreshold);
    SmartDashboard.putNumber("translationThreshold", translationThreshold);
    SmartDashboard.putNumber("distanceThreshold", distanceThreshold);
    SmartDashboard.putNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

    translation = new Translation2d(0, 0);

    //constantSpeedAuto = 0.5;

    //SmartDashboard.putNumber("Constant Speed Auto", constantSpeedAuto);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();

    int desiredTarget = (int) shooterCam.getTargetID();
    if (!Constants.kReefDesiredAngle.containsKey(desiredTarget))
      return;
    desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);

    translation = new Translation2d(0, 0);

    startTime = Timer.getFPGATimestamp();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterCam.setPipeline(drivetrain.getPipelineNumber());
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
    desiredTranslation = SmartDashboard.getNumber("Desired Translation", desiredTranslation);

    //TODO: getHeading is in clockwise positive, should be counterclockwise on 2025j
    rotationError = desiredAngle + drivetrain.getHeading();
    double desiredTx = drivetrain.getHeading() - desiredAngle; // = gyro - desiredAngle
    
    SmartDashboard.putNumber("desired Tx", desiredTx);

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

    double distance = shooterCam.getDistanceEstimatedPose();
    
    double horizontalTranslation = 0;
    double forBackTranslation = 0;
    if (shooterCam.hasTarget()) {
      txValue = shooterCam.getTx();
      
      double translationError = distance * Math.sin((txValue-desiredTx)*(Math.PI/180)) - desiredTranslation;
      
      SmartDashboard.putNumber("Translation Error", translationError);
      SmartDashboard.putNumber("Distance Error", distanceError);
      distanceError = distance - desiredDistance;

      if (Math.abs(translationError) > translationThreshold)
        horizontalTranslation = translationPidController.calculate(translationError) + Math.signum(translationError) * translationFF;

      if (Math.abs(distanceError) > distanceThreshold)
        forBackTranslation = distancePidController.calculate(distanceError) - Math.signum(distanceError) * distanceFF;

      translation = new Translation2d(-forBackTranslation, horizontalTranslation);

      logger.cmdTranslationEntry.append(translationError);
      logger.cmdDistanceEntry.append(distanceError);
    }
    else
      translation = new Translation2d(translation.getX() / 2, translation.getY() / 2);

    SmartDashboard.putNumber("Rotation Error", rotationError);
    logger.cmdRotationEntry.append(rotationError);

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    // calculate forward-backward translation (a dot b)
    // double b1 = -oi.getStrafe();
    // double b2 = oi.getForward();
    // double forBackTranslation = (a1*b1 + a2*b2) * Constants.DriveConstants.kMaxFloorSpeed;

    //constantSpeedAuto = SmartDashboard.getNumber("Constant Speed Auto", constantSpeedAuto);

    logger.cmdCommandXEntry.append(translation.getX());
    logger.cmdCommandYEntry.append(translation.getY());
    logger.cmdCommandRotationEntry.append(rotation);
    
    // double translateX = translation.getX();
    // double translateY = translation.getY();
    //
    // double translateX_sgn = Math.signum(translateX);
    // double translateY_sgn = Math.signum(translateY);
    // double desaturatedX = Math.min(Math.abs(translateX), 1);
    // double desaturatedY = Math.min(Math.abs(translateY), 1);
    // translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

    drivetrain.drive(translation, rotation, false, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double elapsedTime = Timer.getFPGATimestamp() - SmartDashboard.getNumber("Forward start", startTime);
    SmartDashboard.putNumber("time elapsed since start", elapsedTime);

    drivetrain.drive(new Translation2d(0,0), 0, false, null);
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
    //FYI rotationerror will never be >1000, just hacky so it doesn't care about rotation error
    return Math.abs(txValue) < 1 && Math.abs(rotationError) < 1000 && Math.abs(distanceError) < 1 && elapsed >= 0.1;
  }
}
