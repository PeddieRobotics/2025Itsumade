package frc.robot.commands;

//import frc.robot.utils.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.OI;

public class OrbitReef extends Command {

    private Drivetrain drivetrain;
    private OI oi;

    private double turnThreshold, turnFF;
    private PIDController turnPIDController;
    //private Logger logger;
    private double currentHeading, initialHeading;

    private double setpoint;

    public OrbitReef() { //center of the reef is (4.5, 4)!!!
        drivetrain = Drivetrain.getInstance();
        //logger = Logger.getInstance();
        
        SmartDashboard.putNumber("P turn pid orbitReef", 0);
        SmartDashboard.putNumber("I turn pid orbitReef", 0);
        SmartDashboard.putNumber("D turn pid orbitReef", 0);
        SmartDashboard.putNumber("FF turn pid orbitReef", 0);


        turnPIDController = new PIDController(SmartDashboard.getNumber("P turn pid orbitReef", 0), 
                            SmartDashboard.getNumber("I turn pid orbitReef", 0), 
                            SmartDashboard.getNumber("D turn pid orbitReef", 0));

        turnPIDController.enableContinuousInput(-180, 180); //wrap around values 

        turnFF = SmartDashboard.getNumber("FF turn pid orbitReef", 0);

        SmartDashboard.putNumber("robot heading", drivetrain.getHeading());

    }

    @Override
    public void initialize() {
        turnPIDController.setP(SmartDashboard.getNumber("P turn pid orbitReef", 0)); 
        turnPIDController.setI(SmartDashboard.getNumber("I turn pid orbitReef", 0));
        turnPIDController.setD(SmartDashboard.getNumber("D turn pid orbitReef", 0));


    }

    @Override
    public void execute() {
        currentHeading = drivetrain.getHeading();

        setpoint = SmartDashboard.getNumber("CalcReefOffset", drivetrain.calcReefOffset()*(Math.PI/180) - currentHeading);

        double turnToReef = turnPIDController.calculate(currentHeading, setpoint);
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
