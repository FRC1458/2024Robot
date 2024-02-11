package frc.robot.util;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public class MotorUtil {
    public static MotionMagicVelocityVoltage targetSpeed(double speed) {
        return new MotionMagicVelocityVoltage(speed);
    }
}
