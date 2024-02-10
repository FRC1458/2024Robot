package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class IFSManual implements IFS {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    private final XboxController xbox;

    private boolean intakeOn;
    private boolean feederOn;

    
    
    public IFSManual(Shooter shooter, Feeder feeder, Intake intake, XboxController xbox) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;
    }

    @Override
    public void update() {
        if (xbox.getAButtonPressed()){ //toggle intake on/off
            intakeOn = !intakeOn;
            if (intakeOn) {
              intake.slurp();
            }
            else {
              intake.stop();
            }
          }
      
          if(xbox.getLeftTriggerAxis() > 0.7){ //rev up shooter motors, to be changed
            shooter.shoot();
          }
          else{
            shooter.stop();
          }
      
          if(xbox.getRightTriggerAxis() > 0.7){ //"shoot" the piece into the spinning shooter
            feeder.eject();
            intake.slurp();
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


}
