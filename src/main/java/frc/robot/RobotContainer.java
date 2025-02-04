// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
// import frc.robot.commands.AutoAlignment;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LL_PV_Shooter;
// import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.util.OI;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain drivetrain;
  // private LimelightShooter limelightShooter;
  private OI oi;
  private Autonomous autonomous;
  private LL_PV_Shooter LLPVShooter;

  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    drivetrain.setDefaultCommand(new SwerveDriveCommand());

    oi = OI.getInstance();
    LLPVShooter = LL_PV_Shooter.getInstance();
    // oi.configurate();
    
    // limelightShooter = LimelightShooter.getInstance();
    autonomous = Autonomous.getInstance();  

    SmartDashboard.putData("Auto Routines", autonomous.getAutoChooser());
  }

  public Command getAutonomousCommand() {
    return Autonomous.getAutonomousCommand();
  }
}
