package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

public class IFSAuto implements IFS {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    private final XboxController xbox;

    private boolean shootingRunning = false;

    private Timer timerShoot;
    private boolean isShooting;

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;
        timerShoot = new Timer();
        isShooting = false;
        
    }

    @Override
    public void update() {
        if (xbox.getAButton()) { //Turn intake on
            intake.slurp();
          }
        else if (xbox.getYButton()) {
            intake.spit();
        }
          else {
            intake.stop();
          }


        if(xbox.getRightBumper() || xbox.getLeftBumper()) {
            isShooting = true;
            timerShoot.restart();
            if (xbox.getRightBumper()) {
                shooter.scoreSpeakerPID(RobotConstants.shooterSpeedSpeaker);
            }
            else {
                shooter.scoreSpeakerPID(RobotConstants.shooterSpeedAmp);
            }

        }

        if(isShooting && timerShoot.hasElapsed(1)) {
            feeder.feed();
            intake.slurp();
        }

        if(isShooting && timerShoot.hasElapsed(2)){
            stop();
            isShooting = false;
            timerShoot.stop();
        }


    }
    public void stop(){
        shooter.stop();
        feeder.stop();
        intake.stop();
    }
  
}