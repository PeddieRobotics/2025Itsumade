// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class wheelRadiusCharacterization extends Command {
 
  private Drivetrain drivetrain;
  private double currentGyro, initialGyro, deltaGyro;

  /**
   * Creates a new wheelRadiusCharacterization
   .
   *
   * @param subsystem The subsystem used by this command.
   */
  public wheelRadiusCharacterization() {
    drivetrain = Drivetrain.getInstance();
    currentGyro = 0;
    initialGyro = 0;
    deltaGyro = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialGyro = drivetrain.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentGyro = drivetrain.getHeading();
    deltaGyro = currentGyro - initialGyro;
    drivetrain.drive(new Translation2d(0,0), 0.5, false, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
