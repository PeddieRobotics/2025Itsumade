package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightShooter extends Limelight {
    private static LimelightShooter limelightShooter;

    public LimelightShooter() {
        super("limelight-shooter", 0);
    }

    public static LimelightShooter getInstance() {
        if (limelightShooter == null)
            limelightShooter = new LimelightShooter();
        return limelightShooter;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL Distance", getDistance());
        SmartDashboard.putNumber("LL Average Distance", getFilteredDistance());
        SmartDashboard.putNumber("LL ty", getTy());
        SmartDashboard.putNumber("LL tx", getTx());
        SmartDashboard.putString("LL neural class ID", getNeuralClassID());

        // setRedTargetingOffset(SmartDashboard.getNumber("Red Targeting Offset", redTargetOffset));
        // setBlueTargetingOffset(SmartDashboard.getNumber("Blue Targeting Offset", blueTargetOffset));
        //LimelightTarget_Fiducial[] fiducials = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        //SmartDashboard.putNumber("LL results ty", fiducials[0].ty);
        //SmartDashboard.putNumber("LL results ty diff", fiducials[0].ty-getTy());

        updateRollingAverages();
    }
}