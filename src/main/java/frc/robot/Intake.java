package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake {
    private final TalonFX intakeMotor;
   
    public Intake() {
        intakeMotor = new TalonFX(RobotConstants.intakeMotorID);
        intakeMotor.clearStickyFaults();
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
   }

   public void slurp() {
        intakeMotor.set(-RobotConstants.intakeMotorSpeed);
   } //intake note
    public void slurp(double speed) {intakeMotor.set(-speed);} //consistency with feeder (normal speed is slower so note doesn't get stuck as often, remove this method once that problem is fixed)

   public void spit() {
        intakeMotor.set(RobotConstants.intakeMotorSpeed / 2); // play with speeds to find what works
   } //test if this can remove stuck notes from intake

   public void stop() {
        intakeMotor.set(0);
   } //intake doesn't move
}
