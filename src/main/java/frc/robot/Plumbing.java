package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Plumbing {
    private final TalonFX motor;
   
    public Plumbing() {
        motor = new TalonFX(RobotConstants.plumbingMotorID);
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralModeValue.Brake);
   }

   public void eject() {
        motor.set(RobotConstants.plumbingMotorSpeed);
   }

   public void clog(){
        motor.set(0);
   }

}
