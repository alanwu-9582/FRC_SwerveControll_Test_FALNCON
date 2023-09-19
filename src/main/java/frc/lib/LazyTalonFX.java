package frc.lib;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.util.Units;

public class LazyTalonFX extends TalonFX {
    private final double gearRatio;

    public LazyTalonFX(int deviceNumber, double gearRatio) {
        super(deviceNumber);
        // If the motor is for chassis, we won't set a high current limit.
        this.setCurrentLimit(false);
        this.gearRatio = gearRatio;
    }

    public void setCurrentLimit(boolean isLimitHigh) {
        double currentLimit = isLimitHigh ? 35.0 : 25.0;
        double thresholdCurrent = isLimitHigh ? 40.0 : 30.0;
        double thresholdTime = 0.2;
        var supplyConfig = new SupplyCurrentLimitConfiguration(true, currentLimit, thresholdCurrent, thresholdTime);
        var statorConfig = new StatorCurrentLimitConfiguration(true, currentLimit, thresholdCurrent, thresholdTime);
        this.configSupplyCurrentLimit(supplyConfig);
        this.configStatorCurrentLimit(statorConfig);
    }

    public void setRadPosition(double radian) {
        double rotation = Units.radiansToRotations(radian);
        this.setSelectedSensorPosition(rotation * (2048.0 / this.gearRatio));
    }

    public double getVelocityAsMPS(double circumference) {
        double motorRPM = getSelectedSensorVelocity() * (600.0 / 2048.0);
        double mechRPM = motorRPM * this.gearRatio;
        return mechRPM * circumference / 60.0;
    }

    public double getPositionAsRad() {
        double radian = Units.rotationsToRadians(this.getSelectedSensorPosition());
        return radian / (2048.0 / this.gearRatio);
    }
}
