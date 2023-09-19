package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriverJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings("unused")
public class DriverJoystick extends XboxController {
    public static final double DEADBAND = 0.05;

    public DriverJoystick(int port) {
        super(port);
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public Command generateJoystickCommand(SwerveSubsystem swerveSubsystem) {
        return new DriverJoystickCommand(swerveSubsystem, this::getLeftY, this::getLeftX, this::getRightX, this::getBButton);
    }
}
