// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.LimelightShooter;
// import frc.robot.subsystems.SwerveModule;
// import frc.robot.util.Constants.DriveConstants.AutoAlign;
// import frc.robot.util.Constants.DriveConstants.DriveMotor;
// import frc.robot.util.OI;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class AutoAlignment extends Command {
//   private LimelightShooter limelight; 
//   private Drivetrain drivetrain; 
//   private OI oi;

//   private PIDController horizontalAlignPIDController;
//   private double horizontalFF, horizontalThreshold, horizontalInput, txError;

//   public AutoAlignment() {
//     limelight = LimelightShooter.getInstance(); 
//     drivetrain = Drivetrain.getInstance(); 

//     horizontalAlignPIDController = new PIDController(limelight.getAutoP(), AutoAlign.kI, AutoAlign.kD);
//     horizontalFF = limelight.getAutoFF();
//     horizontalThreshold = 1.5;
//     horizontalInput = 0.0;
//     txError = 0.0;



//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(limelight, drivetrain);
//   }


// // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     oi = OI.getInstance();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (limelight.hasTarget()) {
//         txError = limelight.getTxAverage();
//         if (txError < -horizontalThreshold) {
//             horizontalInput = horizontalAlignPIDController.calculate(txError) - horizontalFF;
//         } else if (txError > horizontalThreshold) {
//             horizontalInput = horizontalAlignPIDController.calculate(txError) + horizontalFF;
//         } else {
//             horizontalInput = 0;
//         }
//     }
//     else {
//         horizontalInput = 0;
//     } if (DriverStation.getAlliance().get()==Alliance.Red) {
//         horizontalInput*=-1;
//     }
//     drivetrain.drive(new Translation2d(oi.getForward(), horizontalInput), oi.getRotation(), true, new Translation2d(0,0));
// }


//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // if(txError <= horizontalThreshold){
//     //     return true; 
//     // }
//     return false; 
//   }
// }
