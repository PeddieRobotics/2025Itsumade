package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TimerToDashboard extends Command {
    private String entry;
    
    public TimerToDashboard(String entry) {
        this.entry = entry;
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putNumber(entry, Timer.getFPGATimestamp());
    }
    
    @Override
    public void execute() { }
    
    @Override
    public void end(boolean interrupted) { }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
