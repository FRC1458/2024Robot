package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.StateMachine;
import static frc.robot.IFSAuto.ShootState.*;

public class IFSAuto implements IFS {

    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;
   
    private boolean rampedUp;
    private boolean feederMode;

    private boolean intakeActive;

    public enum ShootState {SPIN_UP, SHOOT}

    private final XboxController xbox1;
    private final XboxController xbox2;

    private Timer timer;
    private boolean intakeOverriden = false;
    private boolean maxSpeed = false;

    private final StateMachine<ShootState> speakerMachine = new StateMachine<>(SPIN_UP);
    private final StateMachine<ShootState> goofyMachine = new StateMachine<>(SPIN_UP);
    private final StateMachine<ShootState> ampMachine = new StateMachine<>(SPIN_UP);
    DigitalInput irBreak;

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox1, XboxController xbox2, DigitalInput irBreak) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox1 = xbox1;
        this.xbox2 = xbox2;
        timer = new Timer();
        feederMode = false;
        this.irBreak = irBreak;

        initStateMachines();


    }

    public void initStateMachines() {
        speakerMachine.addBoolState(SPIN_UP, SHOOT, () -> {
            shooter.shootSpeaker();
            return shooter.shooterRampedUp();
        });
        // speakerMachine.addTimerState(SPIN_UP, 1500, SHOOT, shooter::debug);
        speakerMachine.addOffState(SHOOT,  () -> {
            shooter.shootSpeaker();//shooter.debug();//
            shoot();
        });

        ampMachine.addTimerState(SPIN_UP, 100, SHOOT, shooter::shootAmp);
        ampMachine.addOffState(SHOOT, () -> {
            shooter.shootAmp();
            shootAmp();
        });

        goofyMachine.addTimerState(SPIN_UP, 1500, SHOOT, shooter::shootMax);
        goofyMachine.addOffState(SHOOT, () -> {
            shooter.shootMax();
            shoot();
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

        if(xbox2.getXButtonPressed()) {
            feederMode = !feederMode;
        }
        
        if (xbox2.getYButtonPressed()) {
            if (xbox2.getPOV() == 0) shooter.increaseAmpSpeed();
            if (xbox2.getPOV() == 180) shooter.decreaseAmpSpeed();
        }

        if (xbox1.getRightBumperPressed()) speakerMachine.reset();
        if (xbox1.getRightTriggerAxis() >= 0.7 && !maxSpeed) {
            goofyMachine.reset();
            maxSpeed = true;
        } else if (xbox1.getRightTriggerAxis() < 0.7) maxSpeed = false;
        if (xbox1.getLeftBumperPressed()) ampMachine.reset();

        if(timer.hasElapsed(.75)) {
            rampedUp = true;
        }

        if (xbox1.getRightBumper()){
            speakerMachine.run();
        } else if (xbox1.getRightTriggerAxis() >= 0.7){
            goofyMachine.run();
        } else if (xbox1.getLeftBumper()){
            ampMachine.run();
        } else if(xbox2.getAButton()) {
            timer.start();
            shooter.shootSpeaker();
        } else {
            timer.reset();
            rampedUp = false;
            shooter.stop();
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

        if (xbox2.getBButtonPressed()) intakeOverriden = !intakeOverriden;

        if (xbox1.getAButton() && (intakeOverriden || irBreak.get())) {
            intakeActive = true;  
            intake.slurp();
            feeder.assist();
        }
        else if (xbox1.getYButton()) {
            intakeActive = true;
            intake.spit();
            feeder.reverse();
        }
        else {
            intakeActive = false;
            intake.resetStallState();
            intake.stop();
            feeder.stop();
        }

    }

    @Override
    public boolean isIntakeOverriden() {
        return intakeOverriden;
    }


    
    @Override
    public boolean isIntakeActive() {
        return intakeActive;
    }
}