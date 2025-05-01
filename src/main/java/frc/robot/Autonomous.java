package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;

public class Autonomous {
    private Drivetrain drivetrain;

    private final Command driveAuto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            drivetrain.setStartingPose(new Translation2d(5.0, 3.0));
        }),
        new DriveToPoint(4.0, 3.0, 0, 1.0)
    );

    public final Command waitAuto = new WaitCommand(1);
    
    private static Autonomous autonomous;
    public static Autonomous getInstance() {
        if (autonomous == null)
            autonomous = new Autonomous();
        return autonomous;
    }

    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> autoStartPosition;

    public Autonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("1 meter", driveAuto);
        autoChooser.addOption("Wait", waitAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("NONE/TELEOP", 0.0);
        autoStartPosition.addOption("LEFT", -90.0);
        autoStartPosition.addOption("RIGHT", 90.0);
        autoStartPosition.addOption("CENTER", 180.0);
        autoStartPosition.addOption("RIGHT JIG", 120.0);
        autoStartPosition.addOption("LEFT JIG", -120.0);
        SmartDashboard.putData("Auto Starting Direction", autoStartPosition);

        drivetrain = Drivetrain.getInstance();
    }

    public double getStartHeading() {
        SmartDashboard.putNumber("Selected", autoStartPosition.getSelected());
        return autoStartPosition.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
