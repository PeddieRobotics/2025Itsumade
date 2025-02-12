package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class ForceCalibrate extends Command {
    public ForceCalibrate() { }
    public void initialize() {
        Drivetrain.getInstance().setForcingCalibration(true);
    }
    public void execute() { }
    public void end() {
        Drivetrain.getInstance().setForcingCalibration(false);
    }
    public boolean isFinished() { return false; }
}
