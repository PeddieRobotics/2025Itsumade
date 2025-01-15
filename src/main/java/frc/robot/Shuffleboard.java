package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shuffleboard {
    private static Shuffleboard shuffleboard; 
    private SendableChooser<Command> autoChooser;
    private SendableChooser<String> autoSetupChooser;

    private ComplexWidget autoChooserWidget, autoSetupWidget; 

    private Drivetrain drivetrain; 
    private Autonomous autonomous; 
    public Shuffleboard(){
        drivetrain = Drivetrain.getInstance(); 
        autonomous = Autonomous.getInstance(); 

    }

    public static Shuffleboard getInstance(){
        if(shuffleboard == null){
            shuffleboard = new Shuffleboard(); 
        }
        return shuffleboard; 
    }

    public void configureAutoChooser(){
        SmartDashboard.putData("Auto", autonomous.getAutoChooser()); 
    }

    
}
