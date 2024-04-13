package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder {
    private final TalonFX motor;
    DigitalInput irBreak;
   
    public Feeder(DigitalInput irBreak) {
        this.irBreak = irBreak;
        motor = new TalonFX(RobotConstants.feederMotorID);
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralModeValue.Brake);
   }

   public void feed() {
        motor.set(-RobotConstants.feederMotorSpeed);
   }

   public void feedSlow() {
        motor.set(-RobotConstants.feederMotorSpeed / 3);
   }
   public void reverse() {
        motor.set(RobotConstants.feederMotorSpeed);
   }

   public void stop(){
        motor.set(0);
   }

   public void fullPow(){
     motor.set(-1);
   }

public void assist() {
     if(irBreak.get()) {
        motor.set(-RobotConstants.feederAssistMotorSpeed);
     }
     else{
          stop();
     }
}

public void displayDiagnostics() {
    SmartDashboard.putNumber("Feeder Motor Temp", motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Motor Torque Current", motor.getTorqueCurrent().getValueAsDouble());
}

}
