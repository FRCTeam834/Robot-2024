package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utility.UnitQuad;

/** Stole from 2023 */
public class OI {
    public static final Joystick leftJoystick = new Joystick(0);
    public static final Joystick rightJoystick = new Joystick(1);
    public static final XboxController xbox = new XboxController(2);

    public static final double flightJoystickDeadzone = 0.15;
    public static final double xboxJoystickDeadzone = 0.1;

    /**
     * @return left joystick x input
     */
    public static final double getLeftJoystickX () {
        double raw = leftJoystick.getX();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return left joystick y input
     */
    public static final double getLeftJoystickY () {
        double raw = leftJoystick.getY();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick x input
     */
    public static final double getRightJoystickX () {
        double raw = rightJoystick.getX();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick y input
     */
    public static final double getRightJoystickY () {
        double raw = rightJoystick.getY();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }
    
    /**
     * @return right joystick trigger state
     */
    public static final boolean isRightJoystickTriggerPressed() {
        return rightJoystick.getTrigger();
    }

    public static final double getXboxLeftJoystickY () {
        double raw = xbox.getLeftY();
        if (Math.abs(raw) < xboxJoystickDeadzone) raw = 0.0;
        return raw;
    }

    public static final double getXboxRightJoystickY () {
        double raw = xbox.getRightY();
        if (Math.abs(raw) < xboxJoystickDeadzone) raw = 0.0;
        return raw;
    }
}
