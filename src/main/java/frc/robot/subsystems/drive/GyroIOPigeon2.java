package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    //private final AHRS navX = new AHRS(SPI.Port.kMXP);

    public GyroIOPigeon2 () {
        //navX.reset();
        pigeon = new Pigeon2(18);
        //Pigeon2Configuration config = new Pigeon2Configuration();
        //config.withPigeon2Features(new Pigeon2FeaturesConfigs().withDisableNoMotionCalibration(true));
        //pigeon.getConfigurator().apply(config);
        //pigeon.getConfigurator().setYaw(0.0);
    }

    public void updateInputs (GyroIOInputs inputs) {
        //inputs.yaw = -navX.getYaw();
        //pigeon.getYaw().refresh();
        inputs.yaw = Units.degreesToRadians(pigeon.getYaw().getValue());
    }

    public void resetYaw(double angle){
        // pigeon.setYaw(angle);
    }
}