package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private double[] radianPositions;
    
    private final Pigeon2 gyro;
    private double heading;

    private int pipelineNumber;

    private SwerveDrivePoseEstimator odometry;

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
    
    public Drivetrain() {
        frontLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                    RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID, DriveConstants.kFrontLeftCancoderOffset);
        frontRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                    RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID, DriveConstants.kFrontRightCancoderOffset);
        backLeft = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                    RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID, DriveConstants.kBackLeftCancoderOffset);
        backRight = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                    RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID, DriveConstants.kBackRightCancoderOffset);

        swerveModules = new SwerveModule[] {
            frontLeft, frontRight, backLeft, backRight
        };
        positions = new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
        };

        radianPositions = new double[]{
            frontLeft.getPositionRadians(), frontRight.getPositionRadians(), 
            backLeft.getPositionRadians(), backRight.getPositionRadians()};
        // set states to the values calculated for 0 movement
        states = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        
        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);

        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, getHeadingAsRotation2d(), positions, new Pose2d());

        pipelineNumber = 0;
    }

    public SwerveModulePosition[] getSwerveModulePosition(){
        return positions;
    }

    public double[] getSwerveModuleRadianPosition(){
        return radianPositions;
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

    public double getDegrees(){
        return gyro.getRotation2d().getDegrees(); 
    }
    
    public void updateModulePositions() {
        for (int i = 0; i < 4; i++){
            radianPositions[i] = swerveModules[i].getPositionRadians();

            positions[i] = swerveModules[i].getPosition();
        }
    }
    
    public Pose2d getPose(){
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose){
        gyro.reset();
        odometry.resetPosition(getHeadingAsRotation2d(), positions, pose);
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kinematics.toChassisSpeeds(
                                                          frontLeft.getState(), 
                                                          frontRight.getState(), 
                                                          backLeft.getState(), 
                                                          backRight.getState()
                                                        );
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        states = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        setSwerveModuleStates(states);
    }

    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++)
            swerveModules[i].setDesiredState(desiredStates[i]);
    }

    public void updateOdometry(){
        odometry.update(getHeadingAsRotation2d(), positions);
    }
    
    // in radians/s
    public void drive(Translation2d translation, double rotation,
            boolean fieldOriented, Translation2d centerOfRotation) {

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        
        // not field oriented then joystick direction is robot direction
        ChassisSpeeds robotRelativeSpeeds = fieldRelativeSpeeds;
        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
        
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
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("X acceleration", gyro.getAccelerationX().getValueAsDouble());
        SmartDashboard.putNumber("Y acceleration", gyro.getAccelerationY().getValueAsDouble());
        double pov = OI.getInstance().getDPadPOV();
        if (pov == 90)
            pipelineNumber = 1;
        else if (pov == 270)
            pipelineNumber = 0;

        updateModulePositions();
        updateOdometry();

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

        SmartDashboard.putNumber("module 0 radian position", swerveModules[0].getPositionRadians());

        SmartDashboard.putNumber("gyro angle1", getHeading()); 
        SmartDashboard.putNumber("gyro angle2", getDegrees()); 

        SmartDashboard.putNumber("odometry x", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odometry y", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("odometry angle", odometry.getEstimatedPosition().getRotation().getDegrees());
    }
}
