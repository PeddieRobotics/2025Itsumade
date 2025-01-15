package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.DriveConstants;

public class Autonomous extends SubsystemBase {
    private static SendableChooser<Command> autoChooser; 
    private static Autonomous autonomous; 
    private Drivetrain drivetrain; 
    RobotConfig config;


    public Autonomous(){
        drivetrain = Drivetrain.getInstance(); 
        
        registerNamedCommands(); 
        configureAutoBuilder(); 
        autoChooser = AutoBuilder.buildAutoChooser(); 

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }
    

    public void configureAutoBuilder(){ 
        AutoBuilder.configure(
            drivetrain::getPose,  
            drivetrain::resetPose, 
            drivetrain::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), 
                new PIDConstants(5.0, 0.0, 0.0) 
            ), 
            config,
            () -> {
                var alliance = DriverStation.getAlliance(); 
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return true; 
            },
            drivetrain
        );

    }

    public void registerNamedCommands(){ 
    }

    public static Command getAutonomousCommand(){
        return autoChooser.getSelected(); 
    }

    public SendableChooser<Command> getAutoChooser() { 
        return autoChooser; 
    }

    public static Autonomous getInstance(){
        if (autonomous == null){
            autonomous = new Autonomous(); 
        }
        return autonomous; 
    }

    
    public void driveRobotRelative(ChassisSpeeds speeds){
        drivetrain.driveRobotRelative(speeds);
    }

    
}
