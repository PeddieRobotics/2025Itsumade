package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LL_PV_Shooter extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator ppe;
    private static LL_PV_Shooter ll_pv_shooter;
    private Field2d field;

    public static LL_PV_Shooter getInstance() {
        if (ll_pv_shooter == null)
            ll_pv_shooter = new LL_PV_Shooter();
        return ll_pv_shooter;
    }

    private LL_PV_Shooter() {
        camera = new PhotonCamera("limelight-photonvision-shooter");
        Transform3d robotToCam = new Transform3d(new Translation3d(
            0.37873696, 0, 0.3892028
        ), new Rotation3d(
            0, 0, 0
        ));

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        ppe = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );

        field = new Field2d();
        SmartDashboard.putData("estimatedPhotonVisionRobotPose", field);
    }

    @Override 
    public void periodic() {
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        SmartDashboard.putNumber("pipelineResultLength", pipelineResults.size());

        if (pipelineResults.isEmpty()) {
            return;
        }
        var update = ppe.update(pipelineResults.get(0));
        if (!update.isPresent()) {
            return;
        }
        
        Pose3d estimatedPhotonVisionRobotPose3d = update.get().estimatedPose;
        Pose2d estimatedPhotonVisionRobotPose2d = estimatedPhotonVisionRobotPose3d.toPose2d();
        field.setRobotPose(estimatedPhotonVisionRobotPose2d);
    }  
}
