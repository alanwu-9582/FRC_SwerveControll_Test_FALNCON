package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class GamepadJoystick extends Joystick{
    public GamepadJoystick(int port) {
        super(port);
    }

    public final static int kControllerPort = 0;
    public static final double kDeadband = 0.05;

    public final static int kDriverXAxis = 0;
    public final static int kDriverYAxis = 1;
    public final static int kDriverRotAxis = 4;
    public final static int kDriverFieldOrientedButtonIdx = 6;
}
