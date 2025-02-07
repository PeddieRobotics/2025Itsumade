package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightPVShooter extends PhotonVision {
    private static LimelightPVShooter limelightShooter;

    private LimelightPVShooter() {
        super("front-left-cam", 0.37873696, 0, 0.3892028);
        
    }

    public static LimelightPVShooter getInstance() {
        if (limelightShooter == null)
            limelightShooter = new LimelightPVShooter();
        return limelightShooter;
    }

    @Override
    public void periodic() {
        super.periodic();
    
        SmartDashboard.putNumber("PV Tx", getTx());
        SmartDashboard.putNumber("PV Ty", getTy());
        SmartDashboard.putNumber("PV Average Tx", getTxAverage());
        SmartDashboard.putNumber("PV Average Ty", getTyAverage());
        SmartDashboard.putNumber("PV Number of Targets", getNumberOfTagsSeen());
        SmartDashboard.putBoolean("PV Has Target", hasTarget());
        SmartDashboard.putNumber("PV Target ID", getTargetID());
        SmartDashboard.putNumber("PV Latency", getTotalLatencyInMS());
        SmartDashboard.putNumber("PV Distance (odometry)", getDistanceEstimatedPose());
        SmartDashboard.putNumber("PV Distance (Ty)", getDistanceTy());
        SmartDashboard.putNumber("PV Filtered Distance (odometry)", getFilteredDistanceEstimatedPose());
        SmartDashboard.putNumber("PV Filtered Distane (Ty)", getFilteredDistanceTy());
    }
}