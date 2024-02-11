package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

public class IFSAuto implements IFS {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    private final XboxController xbox;

    private boolean shootingRunning = false;

    private Timer timer;

    public IFSAuto(Shooter shooter, Feeder feeder, Intake intake, XboxController xbox) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;
    }

    @Override
    public void update() {
       if (xbox.getRightTriggerAxis() > 0.7 && !shootingRunning) {
            shootingRunning = true;
            timer.start();
            shooter.scoreSpeakerPID();
       } else if (xbox.getRightTriggerAxis() > 0.7) {
            run();
       } else {
            stop();
            shootingRunning = false;
            timer.stop();
            timer.reset();
       }
    }

    public void run() {
        intake.slurp();
        feeder.feed();
        
    }

    public void stop() {
        shooter.stop();
        intake.stop();
        feeder.stop();
    }
}