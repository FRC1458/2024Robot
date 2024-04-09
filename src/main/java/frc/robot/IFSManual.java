package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IFSManual implements IFS {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    private final XboxController xbox;

    private boolean intakeOn;
    private boolean feederOn;

    
    
    public IFSManual(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;
    }

    @Override
    public void update() {
        if (xbox.getAButton()){ //Turn intake on
          intake.slurp();
        }
        else if (xbox.getYButton()) {
          intake.spit();
        }
        else {
          intake.stop();
        }
          if(xbox.getLeftTriggerAxis() > 0.7){ //rev up shooter motors, to be changed
            shooter.shootSpeaker();
          }
          else if (xbox.getLeftBumper()) {
              shooter.shootAmp();
          }
          else{
            shooter.stop();
          }
      
          if(xbox.getRightTriggerAxis() > 0.7){ //"shoot" the piece into the spinning shooter
            feeder.feed();
            intake.slurp(RobotConstants.feederMotorSpeed);
            feederOn = true;
          }
          else{
            feeder.stop();
            if(feederOn){
              feederOn = false;
              intake.stop();
            }
          }
          
        }

        @Override
        public boolean isRampedUp(){
          return false;
        }

        @Override
        public boolean isIntakeActive() {
          return false;
        }

        @Override
        public boolean isIntakeOverriden() {
            return false;
        }

        @Override
        public boolean isSource(){
          return false;
        }
}
