package frc.robot.subsystems;

public class LimelightFrontMiddle extends Limelight {
    private static LimelightFrontMiddle llFrontMiddle;

    private LimelightFrontMiddle() {
        // super(
        //     CameraConstants.kFrontMiddleCamName,
        //     CameraConstants.kFrontMiddleCamUpOffset,
        //     CameraConstants.kFrontMiddleCamPitchDeg,
        //     false
        // );
        super("limelight-shooter", 0.3892028, 0, true);
    }

    public static LimelightFrontMiddle getInstance() {
        if (llFrontMiddle == null)
            llFrontMiddle = new LimelightFrontMiddle();
        return llFrontMiddle;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}