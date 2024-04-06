package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Timer timer;

    private final StateMachine<ShootState> speakerMachine = new StateMachine<>(SPIN_UP);
    private final StateMachine<ShootState> ampMachine = new StateMachine<>(SPIN_UP);

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox1, XboxController xbox2) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox1 = xbox1;
        this.xbox2 = xbox2;
        timer = new Timer();


        initStateMachines();

        shooterTimer = new Timer();

        intakeActive = false;
    }

    public void initStateMachines() {
        speakerMachine.addTimerState(SPIN_UP, 750, SHOOT, shooter::shootSpeaker);
        speakerMachine.addOffState(SHOOT,  () -> {
            shooter.shootSpeaker();
            shoot();
        });

        ampMachine.addTimerState(SPIN_UP, 100, SHOOT, shooter::shootAmp);
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

    @Override
    public boolean isRampedUp() {
        return rampedUp;
    }

    public void updateShooter() {

        if (xbox1.getRightBumperPressed()) speakerMachine.reset();
        else if (xbox1.getLeftBumperPressed()) ampMachine.reset();

        if(timer.hasElapsed(.75)) {
            rampedUp = true;
        }

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
            timer.start();
            shooter.shootSpeaker();
        }
        else {
            timer.reset();
            rampedUp = false;
            shooter.stop();
        }

        if(xbox2.getYButtonPressed()) {
            shooter.increaseAmpSpeed();
        }
        else if(xbox2.getXButtonPressed()) {
            shooter.decreaseAmpSpeed();
        }
        SmartDashboard.putNumber("Adjusted AMP Shooter Speed", shooter.getAmpSpeed());
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
        if (xbox1.getAButton()) {
            intakeActive = true;
        }
        else {
            intakeActive = false;
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
    
    @Override
    public boolean isIntakeActive() {
        return intakeActive;
    }
}