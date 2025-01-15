package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants.DriveConstants.AutoAlign;

public class LimelightShooter extends Limelight {
    private static LimelightShooter limelightShooter;
    private double autoAlignP; 
    private double autoAlignFF;

    public LimelightShooter() {
        super("limelight-shooter", 0);
        SmartDashboard.putNumber("Horizontal Align P", autoAlignP);
        SmartDashboard.putNumber("Horizontal Align FF", autoAlignFF);
        autoAlignP = 0.05;
        autoAlignFF = 0.09; 
        
    }

    public static LimelightShooter getInstance() {
        if (limelightShooter == null)
            limelightShooter = new LimelightShooter();
        return limelightShooter;
    }

    public double getAutoP(){
        return autoAlignP; 
    }
    public double getAutoFF(){
        return autoAlignFF; 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL Distance", getDistance());
        SmartDashboard.putNumber("LL Average Distance", getFilteredDistance());
        SmartDashboard.putNumber("LL ty", getTy());
        SmartDashboard.putNumber("LL tx", getTx());
        SmartDashboard.putString("LL neural class ID", getNeuralClassID());
        
        autoAlignP = SmartDashboard.getNumber("Horizontal Align P", autoAlignP);
        autoAlignFF = SmartDashboard.getNumber("Horizontal Align FF", autoAlignFF);
       
        

        // setRedTargetingOffset(SmartDashboard.getNumber("Red Targeting Offset", redTargetOffset));
        // setBlueTargetingOffset(SmartDashboard.getNumber("Blue Targeting Offset", blueTargetOffset));
        //LimelightTarget_Fiducial[] fiducials = LimelightHelpers.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        //SmartDashboard.putNumber("LL results ty", fiducials[0].ty);
        //SmartDashboard.putNumber("LL results ty diff", fiducials[0].ty-getTy());

        updateRollingAverages();
        
    }
}