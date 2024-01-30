package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter {
   private final TalonFX motor1;
   private final TalonFX motor2;
   
   
   
    public Shooter() {
        motor1 = new TalonFX(0);
        motor2 = new TalonFX(0);
        motor1.clearStickyFaults();
        motor2.clearStickyFaults();
        motor1.setNeutralMode(NeutralModeValue.Coast);
        motor2.setNeutralMode(NeutralModeValue.Coast);
   }

   public void start() {
        setRPM(0.5);
   }

   public void stop() {
        setRPM(0);
   }

   private void setRPM(double rpm) {
        motor1.set(-1*rpm);
        motor2.set(rpm);
   }

}
