package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.util.OI;

public class AlignmentCommandRobot extends Command {
    private int targetTag = 0;

    private double P = 0.06, I = 0, D = 0, FF = 0;
    private double TxTarget = 0;
    private double TxThreshold = 1;
    
    private Drivetrain drivetrain;
    private OI oi;
    private LimelightShooter limelightShooter;
    private PIDController pidController;
    
    public AlignmentCommandRobot(int targetTag) {
        this.targetTag = targetTag;

        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        pidController = new PIDController(P, I, D);

        addRequirements(drivetrain);
        
        SmartDashboard.putNumber("Alignment P", P);
        SmartDashboard.putNumber("Alignment I", I);
        SmartDashboard.putNumber("Alignment D", D);
        SmartDashboard.putNumber("Alignment FF", FF);
        SmartDashboard.putNumber("Alignment Tx Target", TxTarget);
    }

    public void initialize(){
        oi = OI.getInstance();
        limelightShooter.setPipeline(0);
        limelightShooter.setPriorityTag(targetTag);
        
        // if (Constants.kReefDesiredAngle.containsKey(targetTag))
        //     TxTarget = Constants.kReefDesiredAngle.get(targetTag);
        // else
        //     TxTarget = 0;
    }
    
    public void execute() {
        P = SmartDashboard.getNumber("Alignment P", P);
        I = SmartDashboard.getNumber("Alignment I", I);
        D = SmartDashboard.getNumber("Alignment D", D);
        FF = SmartDashboard.getNumber("Alignment FF", FF);

        SmartDashboard.putNumber("Alignment Tx Target", TxTarget);
        
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);

        if (!limelightShooter.hasTarget())
            return;

        double error = limelightShooter.getTxAverage() - TxTarget;
        
        double translation = 0;
        if (Math.abs(error) > TxThreshold)
            translation = pidController.calculate(error) - Math.signum(error) * FF;
        
        double rotationInput = oi.getRotation();
        double forwardInput = oi.getForward();
        double strafeInput = oi.getForward();

        drivetrain.drive(new Translation2d(0, -translation), rotationInput, false, null);
    }
    
    public boolean isFinished() {
        return !limelightShooter.hasTarget();
    }
    
    public void end(boolean interrupted) {
        drivetrain.drive(new Translation2d(0, 0), 0, false, null);
    }
}

/*
 * PLAN
 * 1. Find Tx
 * 
 */
