package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.StateMachine;
import static frc.robot.IFSAuto.ShootState.*;

public class IFSAuto implements IFS {

    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;
    
    private Timer shooterTimer;
    private boolean rampedUp;

    private boolean intakeActive;

    public enum ShootState {SPIN_UP, SHOOT}

    private final XboxController xbox1;
    private final XboxController xbox2;
    private final StateMachine<ShootState> speakerMachine = new StateMachine<>(SPIN_UP);
    private final StateMachine<ShootState> ampMachine = new StateMachine<>(SPIN_UP);

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox1, XboxController xbox2) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox1 = xbox1;
        this.xbox2 = xbox2;

        initStateMachines();

        shooterTimer = new Timer();

        intakeActive = false;
    }

    public void initStateMachines() {
        speakerMachine.addTimerState(SPIN_UP, 1500, SHOOT, shooter::shootSpeaker);
        speakerMachine.addOffState(SHOOT,  () -> {
            shooter.shootSpeaker();
            shoot();
        });

        ampMachine.addTimerState(SPIN_UP, 1, SHOOT, shooter::shootAmp);
        ampMachine.addOffState(SHOOT, () -> {
            shooter.shootAmp();
            shootAmp();
        });
    }



    @Override
    public void update() {
        updateIntake();
        updateShooter();
    }

    public boolean isRampedUp() {
        return rampedUp;
    }

    public void updateShooter() {
        


        if (xbox1.getRightBumperPressed()) speakerMachine.reset();
        else if (xbox1.getLeftBumperPressed()) ampMachine.reset();

        if (xbox1.getRightBumper()){
            if(rampedUp) {
                shoot();
            }
            else{
                speakerMachine.run();
            }
        }

        else if (xbox1.getLeftBumper()){
            ampMachine.run();
        } 
        
        else if(xbox2.getAButton()) {
            shooter.shootSpeaker();
            shooterTimer.start();
            if(shooterTimer.hasElapsed(1.5)) {
                rampedUp = true;
            }
        }
        
        else {
            shooterTimer.reset();
            rampedUp = false;
            shooter.stop();
        }
    }

    private void shoot() {
        feeder.feed();
        intake.slurp();
    }

    private void shootAmp() {
        feeder.feedSlow();
        intake.slurp();
    }

    private void updateIntake() {
        if (xbox1.getAButtonPressed()) {
            intakeActive = !intakeActive;
        }
        if (intakeActive && Robot.irBreak.get()) {
            intake.slurp();
            feeder.assist();
        }
        else if (xbox1.getYButton()) {
            intake.spit();
            feeder.reverse();
        }
        else if (!Robot.irBreak.get()) {
            intake.stop();
            feeder.stop();
            intakeActive = false;
        }
        else {
            intake.stop();
            feeder.stop();
        }

    }
  
}