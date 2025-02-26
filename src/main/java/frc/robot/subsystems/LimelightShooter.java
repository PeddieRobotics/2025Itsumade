package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightShooter extends Limelight {
    private static LimelightShooter limelightShooter;

    private LimelightShooter() {
        super("limelight-shooter", 0.3892028, 0, true);
        // forward: 0.37873696, left: 0, up: 0.3892028
        
    }

    public static LimelightShooter getInstance() {
        if (limelightShooter == null)
            limelightShooter = new LimelightShooter();
        return limelightShooter;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}