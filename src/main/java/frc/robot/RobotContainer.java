package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick(0);

    public RobotContainer() {
        this.swerveSubsystem.setDefaultCommand(this.driverJoystick.generateJoystickCommand(this.swerveSubsystem));
        this.configureBindings();
    }

    private void configureBindings() {
        this.driverJoystick.b().whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
        // Codes below was deprecated since the update of the API.
        // new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }

    public Command getAutonomousCommand() {
        // TODO implements properly
        return null;
    }
}
