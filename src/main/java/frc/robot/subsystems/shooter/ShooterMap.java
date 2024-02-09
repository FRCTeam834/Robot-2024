package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterMap {
    private final ShooterMapIO io;
    private static InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

    public ShooterMap(ShooterMapIO io) {
        this.io = io;
    }
    
    //These values are not final, perform tests to find proper values
    static {
        table.put(0.0,0.0);
        table.put(213985084.0, Math.PI/2.0);
    }


    public double getShooterAngle(double dist) {
        return table.get(dist);
    }

} 