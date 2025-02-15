package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class DriveConstants {
        public final static double kRriveMotorCurrentLimit = 50;
        public final static double kSteerMotorGearRatio = 150.0 / 7.0;
        public final static double kDriveMotorGearRatio = 6.12;
        public final static double kWheelDiameterIn = 3.82;
        
        // motor rotation to distance traveled by wheel/robot conversion factor
        public static final double kRotorToDistanceRatio = (Units.inchesToMeters(kWheelDiameterIn) * Math.PI) / kDriveMotorGearRatio;

        public class SteerMotor {
            public final static double kP = 135;
            public final static double kI = 0;
            public final static double kD = 7;
            public final static double kS = 0.13;
            public final static double kV = 0;
            public final static double kA = 0;
            public final static double kFF = 0;
            // public static final double kP = 36;
            // public static final double kI = 0.0;
            // public static final double kD = 0.0;
            // public static final double kS = 0.0;
            // public static final double kV = 0.0;
            // public static final double kA = 0.0;
            // public static final double kFF = 0.0;
        }
        public class DriveMotor {
            public final static double kP = 0.5;
            public final static double kI = 0;
            public final static double kD = 0;
            public final static double kS = 0.5;
            public final static double kV = 0.12;
            public final static double kA = 0;
            public final static double kFF = 0;
            // public static final double kS = 0.05;
            // public static final double kV = 0.13;
            // public static final double kA = 0.0;
            // public static final double kP = 0.11;
            // public static final double kI = 0.0;
            // public static final double kD = 0.0;
            // public static final double kFF = 0.0;
        }
        public class AutoAlign {
            public final static double kP = 0.02;
            public final static double kI = 0;
            public final static double kD = 0;
            public final static double kS = 0.5;
            public final static double kV = 0.12;
            public final static double kA = 0;
            public final static double kFF = 0.01;
            // public static final double kS = 0.05;
            // public static final double kV = 0.13;
            // public static final double kA = 0.0;
            // public static final double kP = 0.11;
            // public static final double kI = 0.0;
            // public static final double kD = 0.0;
            // public static final double kFF = 0.0;
        }
        
        public static final double kTrackWidth = Units.inchesToMeters(25.75);
        public static final double kWheelBase = Units.inchesToMeters(22.75);
        
        public static final Translation2d[] kSwerveModuleLocations = {
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            kSwerveModuleLocations[0],
            kSwerveModuleLocations[1],
            kSwerveModuleLocations[2],
            kSwerveModuleLocations[3]
        );
        
        public static final double kFrontLeftCancoderOffset = -2.896;
        public static final double kFrontRightCancoderOffset = -3.073;
        public static final double kBackLeftCancoderOffset = -2.045;
        public static final double kBackRightCancoderOffset = -1.978;
        
        public static final double kMaxFloorSpeed = 3;
    }
    
    // 0 is facing away from driver stations
    // index is april tag #, assuming clockwise
    public static final Map<Integer, Double> kReefDesiredAngle = new HashMap<>() {{
        // red side
        put(6, 60.0);
        put(7, 0.0);
        put(8, -60.0);
        put(9, -120.0);
        put(10, 180.0);
        put(11, 120.0);

        // blue side, same angles but opposite
        put(17, -60.0);
        put(18, 0.0);
        put(19, 60.0);
        put(20, 120.0);
        put(21, 180.0);
        put(22, -120.0);
    }};

    public static final class AutoAlign{
        public static final InterpolatingDoubleTreeMap k1TagStdDevs = new InterpolatingDoubleTreeMap() {{
            put(0.0, 0.1);
            put(0.4, 0.1);
            put(0.53, 0.15);
            put(0.64, 0.25);
            put(0.74, 0.3);
            put(0.8, 0.4);
            put(0.85, 0.5);
            put(0.89, 0.55);
            put(0.93, 0.7);
            put(0.94, 1.2);
            put(1.0, 1.4);
            put(1.1, 1.6);
            put(1.25, 2.5);
        }};

        public static final InterpolatingDoubleTreeMap k2TagStdDevs = new InterpolatingDoubleTreeMap() {{
            put(0.0, 0.3);
            put(1.5, 0.5);
            put(2.5, 0.7);
            put(3.5, 0.9);
            // put(2.0, 0.2);
            // put(2.5, 0.3);
            // put(3.0, 0.5);
        }};
    }

    public static double kCageDesiredAngle;
}
