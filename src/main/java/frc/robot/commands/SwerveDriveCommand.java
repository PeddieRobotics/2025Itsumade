package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.OI;

public class SwerveDriveCommand extends Command {
    private Drivetrain drivetrain;
    private OI oi;
    
    public SwerveDriveCommand() {
        drivetrain = Drivetrain.getInstance();
        oi = OI.getInstance();
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        Translation2d translation = oi.getSwerveTranslation();
        double rotation = oi.getRotation();
        drivetrain.drive(translation, rotation, true, new Translation2d(0, 0));
}
    
    @Override
    public void end(boolean interrupted) {

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
