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
    private Pose2d estimatedPose;

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
        
        estimatedPose = new Pose2d();

        field = new Field2d();
        result = new PhotonPipelineResult();

        SmartDashboard.putData("PhotonVision estimated pose", field);

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

        getEstimatedPoseInternal();
        field.setRobotPose(estimatedPose);
    }

    // ========================================================
    //                 Pose/Translation Getters
    // ========================================================
    
    // PLEASE PLEASE do not call this function yourself
    // the PhotonPoseEstimator only occasionally has updates
    // the public getEstimatedPose returns the latest update
    private void getEstimatedPoseInternal(){
        if (!hasTarget())
            return;

        var update = ppe.update(result);
        if (!update.isPresent())
            return;
        
        estimatedPose = update.get().estimatedPose.toPose2d();
    }
    
    // you should use this for estimated pose
    public Pose2d getEstimatedPose() {
        return estimatedPose;
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

    public double getDistanceEstimatedPose() {
        if (!hasTarget())
            return 0;

        var aprilTagPose = aprilTagFieldLayout.getTagPose(getTargetID());
        if (!aprilTagPose.isPresent())
            return 0;

        Pose2d tag = aprilTagPose.get().toPose2d();
        Pose2d robotPose = getEstimatedPose();

        double dx = tag.getX() - robotPose.getX();
        double dy = tag.getY() - robotPose.getY();
        
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistanceTy() {
        // TODO: set constants 
        return hasTarget() ? PhotonUtils.calculateDistanceToTargetMeters(
            0.3892028, 1.4859, 0, Math.toRadians(getTy())
        ) : 0;
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

            double dist = getDistanceTy();
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
