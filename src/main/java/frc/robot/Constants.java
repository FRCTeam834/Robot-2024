package frc.robot;

public class Constants {
    public static final RobotMode robotMode = RobotMode.SIMULATION;

    public static enum RobotMode {
        COMPETITION,
        DEVELOPMENT,
        SIMULATION
    }

    public static class IndexerConstants {
        public static final double VOLTAGE_LIMIT = 0.0;

    }

    public static class ClimberConstants {
        public static final double VOLTAGE_LIMIT = 0.0;
        public static final double maxArmHeight = 0; //replace with a real value
        public static final double minArmHeight = 0; //this is the real value
    }
}
