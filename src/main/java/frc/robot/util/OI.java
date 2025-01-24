package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignmentCommandRobot;
import frc.robot.commands.Align;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AngleAlignment;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.wheelRadiusCharacterization;
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
        configurate();
    }

    public void configurate() {
        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));    
        
        
        // Trigger SquareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        // SquareButton.whileTrue(new AlignToReefAuto());

        // Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        // xButton.whileTrue(new AlignToReefAuto());

        // Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        // triangleButton.whileTrue(new Align());

        Trigger oButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        oButton.whileTrue(new wheelRadiusCharacterization());

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

    public double getDPadPOV() {
        return controller.getPOV();
    }
}
