package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlignment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Constants.DriveConstants;

public class OI {
    private static OI instance;
    private PS4Controller controller;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
    
    public OI() {
        controller = new PS4Controller(0);
        
        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.whileTrue(new AutoAlignment());
    }
    
    public double getForward() {
        // return -controller.getLeftY();
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }
    
    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }
    
    public Translation2d getSwerveTranslation() {
        return new Translation2d(
            getForward() * DriveConstants.kMaxFloorSpeed,
            getStrafe() * DriveConstants.kMaxFloorSpeed
        );
    }
    
    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double val = (rightRotation - leftRotation) / 2.0;
        return Math.abs(val) < 0.1 ? 0 : val;
    }
}
