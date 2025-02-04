package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.OI;
import frc.robot.util.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;

    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;

    private final Pigeon2 gyro;
    private double heading;

    private int pipelineNumber;

    private double skidAccelLimit;

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public Drivetrain() {
        frontLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kFrontLeftCancoderOffset);
        frontRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kFrontRightCancoderOffset);
        backLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kBackLeftCancoderOffset);
        backRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kBackRightCancoderOffset);

        swerveModules = new SwerveModule[] {
                frontLeft, frontRight, backLeft, backRight
        };
        positions = new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
        };
        // set states to the values calculated for 0 movement
        states = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);

        pipelineNumber = 0;

        SmartDashboard.putNumber("Skid Accel Limit", 10);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getHeadingAsRotation2d() {
        return gyro.getRotation2d();
    }

    public double getDegrees() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getRotationalVelocity() {
        return gyro.getAngularVelocityZWorld().getValueAsDouble() * Math.PI / 180;
    }

    public void updateModulePositions() {
        for (int i = 0; i < 4; i++)
            positions[i] = swerveModules[i].getPosition();
    }

    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++)
            swerveModules[i].setDesiredState(desiredStates[i]);
    }

    // in radians/s
    public void drive(Translation2d translation, double rotation,
            boolean fieldOriented, Translation2d centerOfRotation) {

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        // not field oriented then joystick direction is robot direction
        ChassisSpeeds robotRelativeSpeeds = fieldRelativeSpeeds;
        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());

        // orbit accel skid limit :)
        Translation2d desiredAccel = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond).minus(getRobotRelativeTranslation());
        if (desiredAccel.getNorm() > skidAccelLimit) {
        desiredAccel = desiredAccel.times(skidAccelLimit / desiredAccel.getNorm());
        Translation2d newTranslation = getRobotRelativeTranslation().plus(desiredAccel);
        robotRelativeSpeeds = new ChassisSpeeds(newTranslation.getX(),
        newTranslation.getY(), robotRelativeSpeeds.omegaRadiansPerSecond);
        }

        SmartDashboard.putNumber("rotation thingy", rotation);
        SmartDashboard.putNumber("rotation speed", robotRelativeSpeeds.omegaRadiansPerSecond);

        states = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SmartDashboard.putNumber("module 0 rotation deg", states[0].angle.getDegrees());
        SmartDashboard.putNumber("module 1 rotation deg", states[1].angle.getDegrees());
        SmartDashboard.putNumber("module 2 rotation deg", states[2].angle.getDegrees());
        SmartDashboard.putNumber("module 3 rotation deg", states[3].angle.getDegrees());

        for (int i = 0; i < 4; i++)
            states[i].optimize(new Rotation2d(swerveModules[i].getCANCoderRadians()));

        setSwerveModuleStates(states);

    }

    public int getPipelineNumber() {
        return pipelineNumber;
    }

    public Translation2d getRobotRelativeTranslation() {
        Translation2d sum = new Translation2d();
        for (SwerveModule module : swerveModules) {
            double speed = module.getActualSpeed();
            double angle = module.getCANCoderRadians();
            sum = sum.plus(new Translation2d(speed * Math.cos(angle), speed * Math.sin(angle)));
        }
        return sum.div(4);
    }

    public Translation2d getFieldRelativeTranslation() {
        return getRobotRelativeTranslation().rotateBy(getHeadingAsRotation2d());
    }

    public boolean isSkidding() {
        double currentRotationalVelocity = getRotationalVelocity();
        ArrayList<Double> vectorX = new ArrayList<Double>();
        ArrayList<Double> vectorY = new ArrayList<Double>();
        double tangentialVelocity = -currentRotationalVelocity * DriveConstants.kBaseRadius;
        SmartDashboard.putNumber("Tangential Velocity", tangentialVelocity);
        double times = 1;

        // SmartDashboard.putData("Skid Rotation", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("SwerveDrive");
        //         double rx, ry;

        //         rx = tangentialVelocity * Math.cos(2.41797);// +.72
        //         ry = tangentialVelocity * Math.sin(2.41797);

        //         builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getCANCoderRadians(), null);
        //         builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getActualSpeed(), null);

        //         builder.addDoubleProperty("Front Right Angle", () -> frontRight.getCANCoderRadians(), null);
        //         builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getActualSpeed(), null);

        //         builder.addDoubleProperty("Back Left Angle", () -> backLeft.getCANCoderRadians(), null);
        //         builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getActualSpeed(), null);

        //         builder.addDoubleProperty("Back Right Angle", () -> backRight.getCANCoderRadians(), null);
        //         builder.addDoubleProperty("Back Right Velocity", () -> backRight.getActualSpeed(), null);

        //         builder.addDoubleProperty("Robot Angle", () -> getHeadingAsRotation2d().getRadians(), null);
        //     }
        // });

        for (SwerveModule module : swerveModules) {
            double x = module.getActualSpeed() * Math.cos(module.getCANCoderRadians());
            double y = module.getActualSpeed() * Math.sin(module.getCANCoderRadians());
            double rx;
            double ry;
            
            //2.41797
            double rotationAngleSkidThing =-0.7236;

            if (times == 1) {
                rx = tangentialVelocity * Math.cos(rotationAngleSkidThing);//
                ry = tangentialVelocity * Math.sin(rotationAngleSkidThing);
            } else if (times == 2) {
                rx = tangentialVelocity * Math.cos(Math.PI - rotationAngleSkidThing);//.
                ry = tangentialVelocity * Math.sin(Math.PI - rotationAngleSkidThing);
            } else if (times == 3) {
                rx = tangentialVelocity * Math.cos( - rotationAngleSkidThing);//
                ry = tangentialVelocity * Math.sin( - rotationAngleSkidThing);
            } else {
                rx = tangentialVelocity * Math.cos(Math.PI + rotationAngleSkidThing);//
                ry = tangentialVelocity * Math.sin(Math.PI + rotationAngleSkidThing);
            }

            vectorX.add(x - rx);
            vectorY.add(y - ry);

            times++;
        }

        double epsilon = 0.2; // :)
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                double distance = Math.sqrt(
                        Math.pow(vectorX.get(i) - vectorX.get(j), 2) + Math.pow(vectorY.get(i) - vectorY.get(j), 2));
                if (distance > epsilon) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        double pov = OI.getInstance().getDPadPOV();
        if (pov == 90)
            pipelineNumber = 1;
        else if (pov == 270)
            pipelineNumber = 0;

        SmartDashboard.putNumber("module 0 desired velocity", swerveModules[0].getDesiredState().speedMetersPerSecond);
        SmartDashboard.putNumber("module 1 desired velocity", swerveModules[1].getDesiredState().speedMetersPerSecond);
        SmartDashboard.putNumber("module 2 desired velocity", swerveModules[2].getDesiredState().speedMetersPerSecond);
        SmartDashboard.putNumber("module 3 desired velocity", swerveModules[3].getDesiredState().speedMetersPerSecond);

        SmartDashboard.putNumber("module 0 actual velocity", swerveModules[0].getActualSpeed());
        SmartDashboard.putNumber("module 1 actual velocity", swerveModules[1].getActualSpeed());
        SmartDashboard.putNumber("module 2 actual velocity", swerveModules[2].getActualSpeed());
        SmartDashboard.putNumber("module 3 actual velocity", swerveModules[3].getActualSpeed());

        SmartDashboard.putNumber("module 0 desired angle", swerveModules[0].getDesiredState().angle.getRadians());
        SmartDashboard.putNumber("module 1 desired angle", swerveModules[1].getDesiredState().angle.getRadians());
        SmartDashboard.putNumber("module 2 desired angle", swerveModules[2].getDesiredState().angle.getRadians());
        SmartDashboard.putNumber("module 3 desired angle", swerveModules[3].getDesiredState().angle.getRadians());

        SmartDashboard.putNumber("module 0 actual angle", swerveModules[0].getCANCoderRadians());
        SmartDashboard.putNumber("module 1 actual angle", swerveModules[1].getCANCoderRadians());
        SmartDashboard.putNumber("module 2 actual angle", swerveModules[2].getCANCoderRadians());
        SmartDashboard.putNumber("module 3 actual angle", swerveModules[3].getCANCoderRadians());

        SmartDashboard.putNumber("gyro angle1", getHeading());
        SmartDashboard.putNumber("gyro angle2", getDegrees());

        Translation2d robotRelativeSpeed = getRobotRelativeTranslation();
        SmartDashboard.putNumber("robot x speed", robotRelativeSpeed.getX());
        SmartDashboard.putNumber("robot y speed", robotRelativeSpeed.getY());

        SmartDashboard.putBoolean("Is Skidding", isSkidding());
        SmartDashboard.putNumber("Rotational Velocity", getRotationalVelocity());

        skidAccelLimit = SmartDashboard.getNumber("Skid Accel Limit", 10);

    //     SmartDashboard.putData("Swerve Drive", new Sendable() {
    //         @Override
    //         public void initSendable(SendableBuilder builder) {
    //             builder.setSmartDashboardType("SwerveDrive");

    //             builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getCANCoderRadians(), null);
    //             builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getActualSpeed(), null);

    //             builder.addDoubleProperty("Front Right Angle", () -> frontRight.getCANCoderRadians(), null);
    //             builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getActualSpeed(), null);

    //             builder.addDoubleProperty("Back Left Angle", () -> backLeft.getCANCoderRadians(), null);
    //             builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getActualSpeed(), null);

    //             builder.addDoubleProperty("Back Right Angle", () -> backRight.getCANCoderRadians(), null);
    //             builder.addDoubleProperty("Back Right Velocity", () -> backRight.getActualSpeed(), null);

    //             builder.addDoubleProperty("Robot Angle", () -> getHeadingAsRotation2d().getRadians(), null);
    //         }
    //     });
    }
}
