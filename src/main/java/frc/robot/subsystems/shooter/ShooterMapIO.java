package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public interface ShooterMapIO {
    @AutoLog
    public static class ShooterMapIOInputs {
        public double distance;
    }

    public default void updateInputs(ShooterMapIOInputs inputs) {}
}