package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake {
    private final TalonFX intakeMotor;
   
    public Intake() {
        intakeMotor = new TalonFX(RobotConstants.intakeMotorID);
        intakeMotor.clearStickyFaults();
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
   }

   public void slurp() {
        intakeMotor.set(RobotConstants.intakeMotorSpeed);
   }

   public void eject() {
        intakeMotor.set(-RobotConstants.intakeMotorSpeed);
   }

   public void stop() {
        intakeMotor.set(0);
   }
}
