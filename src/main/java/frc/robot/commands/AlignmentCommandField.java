// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.LimelightShooter;
// import frc.robot.util.OI;

// public class AlignmentCommandField extends Command {
//     private int targetTag = 0;

//     private double P = 0.04, I = 0, D = 0, FF = 0.15;
//     private double TxTarget = 0;
//     private double TxThreshold = 1;
    
//     // calculate the field oriented vector using formula (cos theta, sin theta)
//     private double[] fieldOrientedVector = {1, 0};

//     private Drivetrain drivetrain;
//     private OI oi;
//     private LimelightShooter limelightShooter;
//     private PIDController pidController;
    
//     public AlignmentCommandField(int targetTag) {
//         this.targetTag = targetTag;

//         drivetrain = Drivetrain.getInstance();
//         limelightShooter = LimelightShooter.getInstance();
//         pidController = new PIDController(P, I, D);

//         addRequirements(drivetrain);
        
//         SmartDashboard.putNumber("Alignment P", P);
//         SmartDashboard.putNumber("Alignment I", I);
//         SmartDashboard.putNumber("Alignment D", D);
//         SmartDashboard.putNumber("Alignment FF", FF);
//         SmartDashboard.putNumber("Alignment Tx Target", TxTarget);
//     }

//     public void initialize(){
//         oi = OI.getInstance();
//         limelightShooter.setPipeline(0);
//         limelightShooter.setPriorityTag(targetTag);
        
//         // if (Constants.kReefDesiredAngle.containsKey(targetTag))
//         //     TxTarget = Constants.kReefDesiredAngle.get(targetTag);
//         // else
//         //     TxTarget = 0;
//     }
    
//     public void execute() {
//         P = SmartDashboard.getNumber("Alignment P", P);
//         I = SmartDashboard.getNumber("Alignment I", I);
//         D = SmartDashboard.getNumber("Alignment D", D);
//         FF = SmartDashboard.getNumber("Alignment FF", FF);

//         SmartDashboard.putNumber("Alignment Tx Target", TxTarget);
        
//         pidController.setP(P);
//         pidController.setI(I);
//         pidController.setD(D);

//         if (!limelightShooter.hasTarget())
//             return;

//         double error = limelightShooter.getTxAverage() - TxTarget;
        
//         double x, y;
//         if (Math.abs(error) > TxThreshold) {
//             double k = pidController.calculate(error) - Math.signum(error) * FF;
//             x = fieldOrientedVector[1] * k;
//             y = -fieldOrientedVector[0] * k;
//         }
//         else {
//             x = 0;
//             y = 0;
//         }
        
//         double rotationInput = oi.getRotation();
//         double forwardInput = oi.getForward();
//         double strafeInput = oi.getForward();
        
//         x += forwardInput;
//         y += strafeInput;

//         drivetrain.drive(new Translation2d(x, y), rotationInput, true, null);
//     }
    
//     public boolean isFinished() {
//         return !limelightShooter.hasTarget();
//     }
    
//     public void end(boolean interrupted) {
//         drivetrain.drive(new Translation2d(0, 0), 0, false, null);
//     }
// }

// /*
//  * PLAN
//  * 1. Find Tx
//  * 
//  */
