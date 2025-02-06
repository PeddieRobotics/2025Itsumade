package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RollingAverage;

public abstract class PhotonVision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator ppe;
    private Field2d field;
    private PhotonPipelineResult result;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;
    private RollingAverage txAverage, tyAverage;
    private LinearFilter distFilter;
    private AprilTagFieldLayout aprilTagFieldLayout;

    protected PhotonVision(String cameraName, double cameraForwardOffset,
                        double cameraLeftOffset, double cameraHeightOffset) {
        camera = new PhotonCamera(cameraName);
        Transform3d robotToCam = new Transform3d(new Translation3d(
            cameraForwardOffset, cameraLeftOffset, cameraHeightOffset
        ), new Rotation3d(
            0, 0, 0
        ));

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        ppe = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );
        
        field = new Field2d();
        result = new PhotonPipelineResult();
        SmartDashboard.putData("estimatedPhotonVisionRobotPose", field);

        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        distFilter = LinearFilter.singlePoleIIR(0.24, 0.02);
    }

    @Override 
    public void periodic() {
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        SmartDashboard.putNumber("pipelineResultLength", pipelineResults.size());

        if (pipelineResults.isEmpty())
            return;

        result = pipelineResults.get(0);
        targets = result.getTargets();
        if (!result.hasTargets())
            return;

        bestTarget = result.getBestTarget();
        updateRollingAverages();
    }

    // ========================================================
    //                 Pose/Translation Getters
    // ========================================================
    
    public Pose2d getEstimatedPose(){
        var update = ppe.update(result);
        if (!update.isPresent()) {
            return new Pose2d();
        }
        
        Pose3d estimatedPhotonVisionRobotPose3d = update.get().estimatedPose;
        Pose2d estimatedPhotonVisionRobotPose2d = estimatedPhotonVisionRobotPose3d.toPose2d();

        field.setRobotPose(estimatedPhotonVisionRobotPose2d);

        return estimatedPhotonVisionRobotPose2d;
    }

    // =======================================================
    //                 T-Something Raw Getters
    // =======================================================
    
    public double getTx() {
        return hasTarget() ? bestTarget.getYaw() : 0;
    }

    public double getTy() {
        return hasTarget() ? bestTarget.getPitch() : 0;
    }

    // ================================================
    //                 Distance Getters
    // ================================================

    public double getDistance() {
        Pose2d curPos = getEstimatedPose();

        int tagNo = getTargetID();
        var thing = aprilTagFieldLayout.getTagPose(tagNo);
        if (!thing.isPresent())
            return 0;
        Pose2d tag = thing.get().toPose2d();
        
        double x = tag.getX() - curPos.getX();
        double y = tag.getY() - curPos.getY();
        
        return Math.sqrt(x * x + y * y);

        // TODO: set constants 
        // return hasTarget() ? PhotonUtils.calculateDistanceToTargetMeters(
        //     0.3892028, 0.308, 0, Math.toRadians(getTy())
        // ) : 0;
        
        // if (!hasTarget()) return 0;
        // return (12.125 - 15.375) / (Math.tan(Math.toRadians(getTy())));
    }

    public double getFilteredDistance(){
        return distFilter.lastValue();
    }

    // ======================================================
    //                 Other AprilTag Getters
    // ======================================================
    
    public int getNumberOfTagsSeen() {
        return hasTarget() ? targets.size() : 0;
    }

    public int getTargetID() {
        return hasTarget() ? (int) bestTarget.getFiducialId() : 0;
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    // ====================================================
    //                 Pipeline Controllers
    // ====================================================

    public int getPipeline() {
        return camera.getPipelineIndex(); 
    }

    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
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

    // ===========================================================
    //                 Rolling Average Controllers
    // ===========================================================
    
    public void updateRollingAverages() {
        if (hasTarget()) {
            txAverage.add(getTx());
            tyAverage.add(getTy());

            double dist = getDistance();
            if (dist != 0)
                distFilter.calculate(dist);
        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
    }

    // ======================================
    //                 Others
   // ======================================

    public double getTotalLatencyInMS(){
        return result.metadata.getLatencyMillis();
    }
}
