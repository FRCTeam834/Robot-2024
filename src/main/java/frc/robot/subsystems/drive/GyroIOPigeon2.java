package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOPigeon2 implements GyroIO {
    // private final Pigeon2 pigeon;
    private final AHRS navX;

    public GyroIOPigeon2 () {
        navX = new AHRS(SPI.Port.kMXP);
        navX.reset();

        // pigeon = new Pigeon2(18, "rio");
        // Pigeon2Configuration config = new Pigeon2Configuration();

        // config.Pigeon2Features.DisableNoMotionCalibration = true;
        // config.Pigeon2Features.DisableTemperatureCompensation = false;
        // config.Pigeon2Features.EnableCompass = false;

        // pigeon.getConfigurator().apply(config);
        // pigeon.getYaw().setUpdateFrequency(100);
        // pigeon.getConfigurator().setYaw(0.0);
    }

    public void updateInputs (GyroIOInputs inputs) {
        inputs.yaw = Math.IEEEremainder(-navX.getAngle(), 360);
    }

    public void resetYaw(double angle){
        // pigeon.setYaw(angle);
    }
}