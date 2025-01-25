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

  //PID setpoints and deadbands
  private double desiredAngle;
  private double angleDeadband;
  private double desiredDistance;
  private double distanceDeadband;

  //Weird rotation lerp idea
  private double minRotationLerp, maxRotationLerp;

  //Mechanism values
  private double tx, distance;
  private double mechanismResponseTime = 0; //pose lerping constant

  //Timeout variables
  private int timeoutFrameThreshold = 10;
  private int framesNotSeen;
  
  public DirectAlign() {
    drivetrain = Drivetrain.getInstance();
    limelightShooter = LimelightShooter.getInstance();

    desiredAngle = 0;
    desiredDistance = 25.0;
    framesNotSeen = 0;

    SmartDashboard.putNumber("Desired Distance", desiredDistance);
    
    translationP = 0.05;
    translationI = 0;
    translationD = 0;
    translationFF = 0.01;
    translationPidController = new PIDController(translationP, translationI , translationD);

    SmartDashboard.putNumber("Translation P", translationP);
    SmartDashboard.putNumber("Translation I", translationI);
    SmartDashboard.putNumber("Translation D", translationD);
    SmartDashboard.putNumber("Translation FF", translationFF);

    rotationP = 0.01;
    rotationI = 0.0;
    rotationD = 0.0;
    rotationFF = 0.01;
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

    minRotationLerp = 30;
    maxRotationLerp = 50;

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

    // set PID controllers
    rotationPidController.setPID(rotationP,rotationI,rotationD);
    translationPidController.setPID(translationP, translationI, translationD);

    
    if(limelightShooter.hasTarget()){ 
      framesNotSeen=0;
      tx = limelightShooter.getTx();
      distance = limelightShooter.getFilteredDistance();
    } else {// in increasing order of complexity: use last value, keep rate of change, recalculate according to robot speeds
      framesNotSeen++;
    }

    //lerp rotation
    double currentRotation = drivetrain.getHeading() + drivetrain.getRotationalVelocity()*mechanismResponseTime;
    double rotationError = desiredAngle - currentRotation;


    double rotation;
    rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    //Translation pose lerping - not 'real' pose lerping, only using component parallel to tx
    Translation2d robotRelativeSpeed = drivetrain.getRobotRelativeTranslation();
    double dx = robotRelativeSpeed.getX() * mechanismResponseTime;
    double dy = robotRelativeSpeed.getY() * mechanismResponseTime;

    dx = dx*Math.cos(tx);
    dy = dy*Math.sin(tx);

    distance = distance - Math.sqrt(dx*dx + dy*dy);

    //does this even work lmao
    /*recalculate for orthogonal component, real pose lerping (for distance)

    dx = robotRelativeSpeed.getX() * mechanismResponseTime;
    dy = robotRelativeSpeed.getY() * mechanismResponseTime;

    dx = dx-dx*Math.cos(tx);
    dy = dy-dy*Math.sin(tx);

    distance = Math.sqrt(dx*dx+dy*dy+distance*distance);
    */

    double translationError = distance-desiredDistance;
    Translation2d translationVector = new Translation2d(-translationPidController.calculate(translationError)+Math.signum(translationError)*translationFF,0);
    translationVector=translationVector.rotateBy(Rotation2d.fromDegrees(tx));

    //horizontal thing
    double desiredTx = -(desiredAngle - drivetrain.getHeading());

    if (Math.abs(tx) < 3) {
      translationError = tx - desiredTx;
    } else {
      translationError = distance * Math.sin((tx-desiredTx)*(Math.PI/180));
    }

    double horizontalTranslation = translationPidController.calculate(translationError) - Math.signum(translationError) * translationFF;

    translationVector = translationVector.plus(new Translation2d(0,-horizontalTranslation));


    double farLerp = (currentRotation-minRotationLerp)/(maxRotationLerp-minRotationLerp);
    double closeLerp = 1-farLerp;
    if (Math.abs(rotationError) > rotationThreshold){
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;
    
      //*rotation correction, lerping idea to avoid tag dropout
      if(Math.signum(rotationError)==Math.signum(tx)){
        if(Math.abs(tx)>22.5){
          rotation = 0;//Math.signum(tx)*.1
        }else if(minRotationLerp<distance && distance<maxRotationLerp){
          rotation = closeLerp*rotationPidController.calculate(rotationError) + farLerp*rotationPidController.calculate(tx);
          rotation += Math.signum(rotation) * rotationFF;
          if(Math.signum(rotation)==Math.signum(rotationError)){
            rotation = 0;
          }
        }
      }
      //*/
    }

    drivetrain.drive(translationVector,rotation,false,null);
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
