package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class DriveConstants {
        public final static double kRriveMotorCurrentLimit = 50;
        public final static double kSteerMotorGearRatio = 150.0 / 7.0;
        public final static double kDriveMotorGearRatio = 6.12;
        public final static double kWheelDiameterIn = 4.0;
        
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
        }
        public class DriveMotor {
            public final static double kP = 0.5;
            public final static double kI = 0;
            public final static double kD = 0;
            public final static double kS = 0.5;
            public final static double kV = 0.12;
            public final static double kA = 0;
            public final static double kFF = 0;
        }
     
        public static final double kTrackWidth = 2;
        public static final double kWheelBase = 2;
        
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
        
        public static final double kFrontLeftCancoderOffset = -2.934505247225232;
        public static final double kBackLeftCancoderOffset = -2.060136;

        public static final double kFrontRightCancoderOffset = 3.313392;
        public static final double kBackRightCancoderOffset = -1.992641;
        
        public static final double kMathFloorSpeed = 3;
    }
}
