package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.RollingAverage;

public abstract class Limelight extends SubsystemBase {
    private String name;
    private RollingAverage txAverage, tyAverage, taAverage, rotationAverage, rxAverage, ryAverage;

    private LinearFilter distFilter;
    private double lastDistance;

    private double redTargetOffset, blueTargetOffset;

    private Pose2d calculatedBotpose;

    // protected -- allow descendants to use but not others
    protected Limelight(String name, int pipeline) {
        this.name = name;
        setPipeline(pipeline);

        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        taAverage = new RollingAverage();
        rotationAverage = new RollingAverage();
        rxAverage = new RollingAverage(); 
        ryAverage = new RollingAverage(); 

        distFilter = LinearFilter.singlePoleIIR(0.24, 0.02);
        lastDistance = 0;
    }

    // ==================================================
    //                 T-Something Getters
    // ==================================================
    
    // Tv is whether the limelight has a valid target
    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }

    // Tx is the Horizontal Offset From Crosshair To Target
    public double getTx() {
        double value = LimelightHelpers.getTX(name);
        return value;
    }

    // Ty is the Vertical Offset From Crosshair To Target
    public double getTy() {
        double value = LimelightHelpers.getTY(name);
        return value;
    }

    // TODO: what the hell does this do???
    // public double getTx_NoCrosshair() {
    //     return LimelightHelpers.getTX_NoCrosshair(name);
    // }
    //
    // public double getTy_NoCrosshair() {
    //     return LimelightHelpers.getTY_NoCrosshair(name);
    // }

    public double getTa() {
        return LimelightHelpers.getTA(name);
    }

    // ===============================================
    //                 Average Getters
    // ===============================================

    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    public double getTaAverage() {
        return taAverage.getAverage();
    }

    public double getRotationAverage() {
        return rotationAverage.getAverage();
    }

    public double getRXAverage(){
        return rxAverage.getAverage(); 
    }

    public double getRYAverage(){
        return ryAverage.getAverage(); 
    }
    
    // ================================================
    //                 Distance Getters
    // ================================================

    // TODO: implement
    public double getDistance() {
        if (!hasTarget()) {
            return 0;
        } else {
            // a1 = LL panning angle
            // a2 = additional angle to target
            // tan(a1 + a2) = h/d
            // d = h/tan(a1+a2)
            // return (LimelightConstants.kSpeakerAprilTagHeight - LimelightConstants.kLimelightHeight) /
            //         (Math.tan(Math.toRadians(LimelightConstants.kLimelightPanningAngle + getTy())));
            return (30.5 - 17.88) /
                    (Math.tan(Math.toRadians(25.0 - getTy())));
            // return 0;
        }
    }

    public double getLastDistance(){
        return lastDistance;
    }

    public double getDistanceForLogging() {
        if (hasTarget())
            return getDistance();
        return getLastDistance();
    }

    public double getFilteredDistance(){
        return distFilter.lastValue();
    }

    // ================================================
    //                 Apriltag Getters
    // ================================================
    
    // Class ID of primary neural detector result or neural classifier result
    public String getNeuralClassID() {
        return LimelightHelpers.getNeuralClassID(name);
    }

    public int getNumberOfTagsSeen() {
        double[] botposeArr = LimelightHelpers.getBotPose_wpiBlue(name);
        return botposeArr.length == 0 ? 0 : (int) botposeArr[7];
    }

    public boolean hasTarget() {
        return getTv();
    }

    public double getTargetID() {
        return LimelightHelpers.getFiducialID(name);
    }

    // ============================================
    //                 Other Getters
    // ============================================

    public void setRedTargetingOffset(double offset){
        redTargetOffset = offset;
    }

    public void setBlueTargetingOffset(double offset){
        blueTargetOffset = offset;
    }

    public double getRedTargetingOffset(){
        return redTargetOffset;
    }

    public double getBlueTargetingOffset(){
        return blueTargetOffset;
    }
    
    // ===============================================================
    //                 Rolling Average Control Methods
    // ===============================================================
    
    public void updateRollingAverages() {
        if (hasTarget()) {
            txAverage.add(getTx());
            tyAverage.add(getTy());
            taAverage.add(getTa());

            double dist = getDistance();
            if (dist != 0)
                distFilter.calculate(dist);
            
            // rotationAverage.add(getBotpose().getRotation().getDegrees());//based on alliance of driverstation, awaiting testing 
            rotationAverage.add(getMT1BotPose().getRotation().getDegrees()); 

            //red
            rxAverage.add(getMT1BotPose().getX()); 
            ryAverage.add(getMT1BotPose().getY()); 
        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
        taAverage.clear();
        rotationAverage.clear(); 
        rxAverage.clear(); 
        ryAverage.clear(); 
    }

    // =======================================
    //                 Botpose
    // =======================================

    public Pose2d getMT1BotPose() {
        double[] result;
        try {
            // Use blue coordinates by default for all odometry
            result = LimelightHelpers.getBotPose_wpiBlue(name);
            // if(DriverStation.getAlliance().get() == Alliance.Red){
            //     result = LimelightHelpers.getBotPose_wpiRed(name);
            // }
            // else{
            //     result = LimelightHelpers.getBotPose_wpiBlue(name);
            // }
        } catch (java.util.NoSuchElementException e) {
            return new Pose2d();
        }

        if (result.length > 0) {
            return new Pose2d(
                new Translation2d(result[0], result[1]), 
                new Rotation2d(Math.toRadians(result[5]))
            );
        }

        return new Pose2d();
    }
    
    public Pose2d getMT2BotPose() {
        double gyro = Drivetrain.getInstance().getHeading();
        LimelightHelpers.SetRobotOrientation(name, gyro, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        // gyro rate
        return mt2.pose != null ? mt2.pose : new Pose2d();
    }

    // public void checkForAprilTagUpdates(SwerveDrivePoseEstimator odometry) {
    //     int tagsSeen = getNumberOfTagsSeen();
    //     //IMPORTANT:still has safe guard preventing the use of update vision if it is outside a half meter range, delete or change 
    //     //condition to furhter enable checkforapriltag updates 
    //     
    //     // if (tagsSeen > 1 && this.getBotpose().relativeTo(odometry.getEstimatedPosition()).getTranslation().getNorm() < 0.5) {
    //     //     odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
    //     // }
    //
    //     if (tagsSeen > 1) {
    //         // Get pipeline and capture latency (in milliseconds)
    //         double tl = LimelightHelpers.getLatency_Pipeline(name);
    //         double cl = LimelightHelpers.getLatency_Capture(name);
    //
    //         // Calculate a latency-compensated timestamp for the vision measurement (in seconds)
    //         double timestampLatencyComp = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);
    //         calculatedBotpose = this.getMT1BotPose();
    //         odometry.addVisionMeasurement(calculatedBotpose, timestampLatencyComp);
    //     }
    //     else
    //         calculatedBotpose = null;
    // }
    
    // getBotpose returns botpose regardless
    // getCalculatedBotpose only returns values that are used,
    // i.e. those found when >=2 limelights used
    public Pose2d getCalculatedBotpose() {
        return calculatedBotpose;
    }
    
    // Gets the total latency of the limelight capture + pipeline processing for the current image, in milliseconds (MS)
    public double getTotalLatencyInMS(){
        double tl = LimelightHelpers.getLatency_Pipeline(name);
        double cl = LimelightHelpers.getLatency_Capture(name);
        return tl + cl;
    }

    // ======================================
    //                 Others
    // ======================================
    
    public void setPriorityTag(int tagID) {
        LimelightHelpers.setPriorityTagID(name, tagID);
    }

    public void setPipeline(int pipelineNum) {
        LimelightHelpers.setPipelineIndex(name, pipelineNum);
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(name);
    }

    public String getJSONDump() {
        return LimelightHelpers.getJSONDump(name);
    }
}

