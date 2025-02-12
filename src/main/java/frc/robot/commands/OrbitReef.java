package frc.robot.commands;

//import frc.robot.utils.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Constants.DriveConstants.AutoAlign;
import frc.robot.util.OI;

public class OrbitReef extends Command {

    private Drivetrain drivetrain;
    private OI oi;

    private double P = 0.12, I = 0, D = 0, FF = 0;

    private double turnThreshold, turnFF;
    private PIDController turnPIDController;
    private double currentHeading, initialHeading;

    private double setpoint;

    public OrbitReef() { //center of the reef is (4.5, 4)!!!
        drivetrain = Drivetrain.getInstance();
        
        SmartDashboard.putNumber("P turn pid orbitReef", 0);
        SmartDashboard.putNumber("I turn pid orbitReef", 0);
        SmartDashboard.putNumber("D turn pid orbitReef", 0);
        SmartDashboard.putNumber("FF turn pid orbitReef", 0);

        turnPIDController = new PIDController(0, 0, 0);
        turnPIDController.enableContinuousInput(-180, 180); //wrap around values 

        turnFF = SmartDashboard.getNumber("FF turn pid orbitReef", 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        oi = OI.getInstance();
    }

    @Override
    public void execute() {
        turnPIDController.setP(SmartDashboard.getNumber("P turn pid orbitReef", 0)); 
        turnPIDController.setI(SmartDashboard.getNumber("I turn pid orbitReef", 0));
        turnPIDController.setD(SmartDashboard.getNumber("D turn pid orbitReef", 0));
        turnFF = SmartDashboard.getNumber("FF turn pid orbitReef", turnFF);
        
        currentHeading = drivetrain.getHeading();

        double currentHeading = drivetrain.getHeading();
        double poseX = drivetrain.getPose().getX();
        double poseY = drivetrain.getPose().getY();
        // setpoint = Math.atan((AutoAlign.reefCenterY - poseY) / (AutoAlign.reefCenterX - poseX));
        setpoint = Math.atan2(AutoAlign.reefCenterY - poseY, AutoAlign.reefCenterX - poseX);
        setpoint = Math.toDegrees(setpoint);

        SmartDashboard.putNumber("orbit reef error", drivetrain.getHeading() - setpoint);
        SmartDashboard.putNumber("orbit reef setpoint", setpoint);

        double turnToReef = turnPIDController.calculate(currentHeading, setpoint);
        //turnToReef + turnFF * Math.signum(turnToReef)
        drivetrain.drive(oi.getSwerveTranslation(), turnToReef + turnFF * Math.signum(turnToReef), true, new Translation2d(0, 0));

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(currentHeading) - setpoint) < turnThreshold;
    }

}
