package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drivetrain.ShooterIOInputsAutoLogged;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double roller0Velocity;
        public double roller1Velocity;
        public double pivotAngle;
        public double pivotVelocity;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setShooterVoltage(double voltage) {}
    public default void setPivotVoltage(double voltage) {}
}