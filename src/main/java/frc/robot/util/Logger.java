package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightShooter;

public class Logger {
    private static Logger instance;
    public DoubleLogEntry cmdTranslationEntry, cmdDistanceEntry, cmdRotationEntry, cmdCommandXEntry, cmdCommandYEntry, cmdCommandRotationEntry;
    public DoubleLogEntry deltaGyro, averageDeltaPositions, effectiveWheelRadius;
    private DataLog log = DataLogManager.getLog();
    private Pose2d fieldPosition;

    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        // Superstructure Logs
        cmdTranslationEntry = new DoubleLogEntry(log, "Align Command/Translation Error");
        cmdDistanceEntry = new DoubleLogEntry(log, "Align Command/Distance Error");
        cmdRotationEntry = new DoubleLogEntry(log, "Align Command/Rotation Error");
        cmdCommandXEntry = new DoubleLogEntry(log, "Align Command/Commanded X");
        cmdCommandYEntry = new DoubleLogEntry(log, "Align Command/Commanded Y");
        cmdCommandRotationEntry = new DoubleLogEntry(log, "Align Command/Commanded Rotation");
        deltaGyro = new DoubleLogEntry(log, "Characterization Command/deltaGyro");
        averageDeltaPositions = new DoubleLogEntry(log, "Characterization Command/averageDeltaPositions");
        effectiveWheelRadius = new DoubleLogEntry(log, "Characterization Command/effectiveWheelRadius");
    }

    public void logEvent(String event, Boolean isStart) {
        // commandEntry.append(event + (isStart? " Started": " Ended"));

    }

    public void updateLogs() {

        // Drivetrain
        // updateDrivetrainLogs();

        // Intake

        //Arm

        //limelight
        // LLShooterDistanceEntry.append(limelightShooter.getDistance());
        // LLShooterTxEntry.append(limelightShooter.getTx());

    }
}