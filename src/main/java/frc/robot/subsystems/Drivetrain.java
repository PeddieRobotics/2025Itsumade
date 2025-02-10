package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.OI;
import frc.robot.util.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;
    private static LimelightPVShooter limelightPVShooter;
    
    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;
    
    private final Pigeon2 gyro;
    private double heading;

    private int pipelineNumber;

    private SwerveDrivePoseEstimator odometry;

    private final Field2d fusedOdometryPose;

    // private LimelightShooter limelightShooter;

    private boolean isForcingCalibration;
    private boolean useMegaTag = true;

    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }
    
    public Drivetrain() {
        fusedOdometryPose = new Field2d();
        SmartDashboard.putData("fused odometry", fusedOdometryPose);

        SmartDashboard.putBoolean("isForcingCalibration", isForcingCalibration);
        SmartDashboard.putBoolean("useMegaTag", useMegaTag);
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
        // set states to the values calculated for 0 movement
        states = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        
        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);

        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, getHeadingAsRotation2d(), positions, new Pose2d());

        pipelineNumber = 0;

        // limelightShooter = LimelightShooter.getInstance();

        isForcingCalibration = true;
        
        limelightPVShooter = LimelightPVShooter.getInstance();
    }
    
    public void resetGyro() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }
    
    public Rotation2d getHeadingAsRotation2d() {
        return gyro.getRotation2d();
    }

    public void updateModulePositions() {
        for (int i = 0; i < 4; i++)
            positions[i] = swerveModules[i].getPosition();
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

    public void forceCalibrate(boolean force) {
        isForcingCalibration = force;
    }

    public void updateOdometry(){
        odometry.update(getHeadingAsRotation2d(), positions);
        int numAprilTag = limelightPVShooter.getNumberOfTagsSeen();

        if (isForcingCalibration) {
            limelightPVShooter.checkForAprilTagUpdates(odometry);
            
            // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(
            //     0.01, 0.01, 70
            // ));
    
            // // odometry set devs
            // double latency = limelightPVShooter.getTotalLatencyInMS();
            // double timestampLatencyComp = Timer.getFPGATimestamp() - latency / 1000.0;
    
            // Pose2d estimatedPose = limelightPVShooter.getEstimatedPose();
    
            // odometry.addVisionMeasurement(estimatedPose, timestampLatencyComp);
            
            // isForcingCalibration = false;
        }
        

        //  if(DriverStation.isAutonomous()){
        //     if (isForcingCalibration) {
        //         limelightShooter.checkForAprilTagUpdates(odometry);
        //     }
        // }
        // else{
        //     if (useMegaTag || isForcingCalibration) {
        //         limelightShooter.checkForAprilTagUpdates(odometry);
        //         isForcingCalibration = false;
        //     }
        // }
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
    
    public double calcReefOffset(){
        double currentHeading = getHeading();
        double poseX = getPose().getX();
        double poseY = getPose().getY();
    
        // return Math.atan((FieldConstants.reefY - poseY) / (FieldConstants.reefX - poseX));
        return 0;
    }

    @Override
    public void periodic() {

        // SmartDashboard.getBoolean("isForcingCalibration", isForcingCalibration);
        // SmartDashboard.getBoolean("useMegaTag", useMegaTag);

        double pov = OI.getInstance().getDPadPOV();
        if (pov == 90)
            pipelineNumber = 1;
        else if (pov == 270)
            pipelineNumber = 0;

        // if (limelightShooter.getNumberOfTagsSeen() >= 2
        //     // && (useMegaTag || isForcingCalibration)
        // ) {
        //     Matrix<N3, N1> visionStdDevs = VecBuilder.fill(
        //         isForcingCalibration ? 0.0001 : 1,
        //         isForcingCalibration ? 0.0001 : 1,
        //         isForcingCalibration ? 0.0001 : 30
        //     );
        //     odometry.setVisionMeasurementStdDevs(visionStdDevs);
        //
        //     // odometry set devs
        //     double pl = LimelightHelpers.getLatency_Pipeline(limelightShooter.getName());
        //     double cl = LimelightHelpers.getLatency_Capture(limelightShooter.getName());
        //     
        //     double timestampLatencyComp = Timer.getFPGATimestamp() - (pl/1000.0) - (cl/1000.0);
        //     Pose2d botpose = limelightShooter.getMT1BotPose();
        //
        //     odometry.addVisionMeasurement(botpose, timestampLatencyComp);
        // }

        updateModulePositions();
        updateOdometry();

        fusedOdometryPose.setRobotPose(odometry.getEstimatedPosition());

        // mt1BotposePose.setRobotPose(limelightShooter.getMT1BotPose());
        //
        // LimelightHelpers.SetRobotOrientation("limelight-shooter",
        //     odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
        //
        // mt2BotposePose.setRobotPose(mt2.pose);
        // odometryPose.setRobotPose(odometry.getEstimatedPosition());

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

        SmartDashboard.putNumber("gyro heading", getHeading()); 

        SmartDashboard.putNumber("odometry x", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odometry y", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("odometry angle", odometry.getEstimatedPosition().getRotation().getDegrees());
    }
}
