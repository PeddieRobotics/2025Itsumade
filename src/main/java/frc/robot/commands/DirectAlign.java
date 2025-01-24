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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DirectAlign extends Command {
  private Drivetrain drivetrain;
  private LimelightShooter limelightShooter;

  //PID controllers
  private PIDController rotationPidController, translationPidController;
  private double rotationUseLowerPThreshold, rotationThresholdP;
  private double translationThreshold, rotationThreshold, distanceThreshold;
  private double translationP, translationI, translationD, translationFF;
  private double rotationP, rotationI, rotationD, rotationFF;

  //PID setpoints
  private double desiredAngle;
  private double desiredDistance;

  //Mechanism values
  private double tx, distance;
  private double mechanismResponseTime = .1; //pose lerping constant

  //Timeout variables
  private int timeoutFrameThreshold = 10;
  private int framesNotSeen;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DirectAlign() {
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();

    desiredAngle = 0;
    desiredDistance = 25.0;
    framesNotSeen = 0;

    SmartDashboard.putNumber("Desired Distance", desiredDistance);
    
    translationP = 0.04;
    translationI = 0;
    translationD = 0;
    translationFF = 0.02;
    translationPidController = new PIDController(translationP, translationI , translationD);

    SmartDashboard.putNumber("Translation P", translationP);
    SmartDashboard.putNumber("Translation I", translationI);
    SmartDashboard.putNumber("Translation D", translationD);
    SmartDashboard.putNumber("Translation FF", translationFF);

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

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    int desiredTarget = (int) limelightShooter.getTargetID();
    if (!Constants.kReefDesiredAngle.containsKey(desiredTarget))
      return;

    desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelightShooter.setPipeline(drivetrain.getPipelineNumber());
    //live update PIDs and PID setpoints
    translationP = SmartDashboard.getNumber("Translation P", translationP);
    translationI = SmartDashboard.getNumber("Translation I", translationI);
    translationD = SmartDashboard.getNumber("Translation D", translationD);
    translationFF = SmartDashboard.getNumber("Translation FF", translationFF);

    rotationP = SmartDashboard.getNumber("Rotation P", rotationP);
    rotationThresholdP = SmartDashboard.getNumber("Rotation Threshold P", rotationThresholdP);
    rotationI = SmartDashboard.getNumber("Rotation I", rotationI);
    rotationD = SmartDashboard.getNumber("Rotation D", rotationD);
    rotationFF = SmartDashboard.getNumber("Rotation FF", rotationFF);

    rotationThreshold = SmartDashboard.getNumber("rotationThreshold", rotationThreshold);
    translationThreshold = SmartDashboard.getNumber("translationThreshold", translationThreshold);
    distanceThreshold = SmartDashboard.getNumber("distanceThreshold", distanceThreshold);

    desiredDistance = SmartDashboard.getNumber("Desired Distance", desiredDistance);

    double rotationError = desiredAngle - drivetrain.getHeading();
    double desiredTx = -rotationError;

    // set PID controllers
    rotationPidController.setPID(rotationP,rotationI,rotationD);
    translationPidController.setPID(translationP, translationI, translationD);

    
    if(limelightShooter.hasTarget()){ 
      framesNotSeen=0;
      tx = limelightShooter.getTx();
      distance = limelightShooter.getFilteredDistance();
      SmartDashboard.putNumber("LL distance",distance);
    } else {// in increasing order of complexity: use last value, keep rate of change, recalculate according to robot speeds
      framesNotSeen++;
    }

    double translationError = distance-desiredDistance;
    SmartDashboard.putNumber("Translation error", translationError);
    Translation2d translationVector = new Translation2d(-translationPidController.calculate(translationError)+Math.signum(translationError)*translationFF,0);
    translationVector=translationVector.rotateBy(Rotation2d.fromDegrees(tx));

    drivetrain.drive(translationVector,0,false,null);
    
    /* 
    double horizontalTranslation = 0;
    double forBackTranslation = 0;
    if (limelightShooter.hasTarget()) {
      double txValue = limelightShooter.getTx();
      double translationError = 0;
      if (Math.abs(txValue) < 3) {
        translationError = txValue - desiredTx;
      } else {
        translationError = distance * Math.sin((txValue-desiredTx)*(Math.PI/180));
      }
      SmartDashboard.putNumber("Translation Error", translationError);
      double distanceError = distance - desiredDistance;

      if (Math.abs(translationError) > translationThreshold)
        horizontalTranslation = translationPidController.calculate(translationError) - Math.signum(translationError) * translationFF;

      if (Math.abs(distanceError) > distanceThreshold)
        forBackTranslation = distancePidController.calculate(distanceError) - Math.signum(distanceError) * distanceFF;
      
    }

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;



    drivetrain.drive(new Translation2d(-forBackTranslation, -horizontalTranslation), rotation, false, null);
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(0,0), 0, false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return framesNotSeen>=timeoutFrameThreshold;
  }
}
