package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoTrackCmd;
import frc.robot.commands.DriverJoystickCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.joysticks.DriverJoystick;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.XboxController;


public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final DriverJoystick driverJoystick = new DriverJoystick(0);
    private final DriverJoystickCommand driverJoystickCommand = new DriverJoystickCommand(swerveSubsystem, driverJoystick);


    public RobotContainer() {
        this.swerveSubsystem.setDefaultCommand(driverJoystickCommand);
        this.configureBindings();
        this.putToDashboard();
    }

    public void onRobotPeriodic() {
    }

    private void putToDashboard() {

    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(this.swerveSubsystem::zeroHeading));
        new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new AutoTrackCmd(this.swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        return new AutoTrackCmd(swerveSubsystem);
        // return new Auto(swerveSubsystem);
          
    }
}
