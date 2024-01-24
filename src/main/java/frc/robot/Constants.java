package frc.robot;

public class Constants {
    public static final RobotMode robotMode = RobotMode.SIMULATION;

    public static enum RobotMode {
        COMPETITION,
        DEVELOPMENT,
        SIMULATION
    }

    public static class ShooterConstants {
        public static final double PIVOT_GEAR_REDUCTION = 138;
        public static final double MAX_SHOOTER_SPEED = 0.0;
    }

    public static class IndexerConstants {
        public static final double VOLTAGE_LIMIT = 0.0;

    }
}
