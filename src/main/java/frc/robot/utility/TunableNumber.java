package frc.robot.utility;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;

/**
 * Stolen from 6328
 * http://github.com/Mechanical-Advantage
 */

public class TunableNumber {
    private final String key;
    private double defaultValue = 0.0;
    private boolean hasDefaultValue = false;
    private Map<Integer, Double> lastValues = new HashMap<>();

    public TunableNumber (String name) {
        this.key = "Tunable/" + name;
    }

    public void initDefault (double defaultValue) {
        this.defaultValue = defaultValue;
        hasDefaultValue = true;
        if (Constants.robotMode == RobotMode.DEVELOPMENT) {
            SmartDashboard.putNumber(key, 
                SmartDashboard.getNumber(key, defaultValue));
        }
    }

    public double get () {
        if (!hasDefaultValue) {
            throw new RuntimeException("No default value set for TunableNumber with key: " + key);
        }
        if (Constants.robotMode == RobotMode.DEVELOPMENT) {
            return SmartDashboard.getNumber(key, defaultValue);
        } else {
            return defaultValue;
        }
    }

    public boolean hasChanged (int hash) {
        double current = get();
        Double last = lastValues.get(hash);
        if (last == null || current != last) {
            lastValues.put(hash, current);
            return true;
        }
        return false;
    }
}