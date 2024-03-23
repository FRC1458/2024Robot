package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.StateMachine;
import static frc.robot.IFSAuto.ShootState.*;

public class IFSAuto implements IFS {

    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;
    private final Timer feederAssist = new Timer();

    public enum ShootState {SPIN_UP, SHOOT}

    private final XboxController xbox;
    private final StateMachine<ShootState> speakerMachine = new StateMachine<>(SPIN_UP);
    private final StateMachine<ShootState> ampMachine = new StateMachine<>(SPIN_UP);

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;

        initStateMachines();


    }

    public void initStateMachines() {
<<<<<<< Updated upstream
        speakerMachine.addTimerState(SPIN_UP, 750, ANGLE, this::spinUpSpeaker);
        speakerMachine.addBoolState(ANGLE, SHOOT, () -> {
            shooter.shootSpeaker();
            return shooter.pivotPointBlank();
        });
=======
        speakerMachine.addTimerState(SPIN_UP, 1500, SHOOT, shooter::shootSpeaker);
>>>>>>> Stashed changes
        speakerMachine.addOffState(SHOOT,  () -> {
            shooter.shootSpeaker();
            shoot();
        });

        ampMachine.addTimerState(SPIN_UP, 1500, SHOOT, shooter::shootAmp);
        ampMachine.addOffState(SHOOT, () -> {
            shooter.shootAmp();
            shoot();
        });
    }



    @Override
    public void update() {
        updateIntake();
        updateShooter();
    }

    public void updateShooter() {
        if (xbox.getRightBumperPressed()) speakerMachine.reset();
        else if (xbox.getLeftBumperPressed()) ampMachine.reset();

        if (xbox.getRightBumper()) speakerMachine.run();
        else if (xbox.getLeftBumper()) ampMachine.run();
        else shooter.stop();
    }

<<<<<<< Updated upstream
    private void spinUpSpeaker() {
        shooter.shootSpeaker();
        shooter.pivotPointBlank();
    }

    private void spinUpAmp() {
        shooter.shootAmp();
        shooter.pivotToAmp();
    }

=======
>>>>>>> Stashed changes
    private void shoot() {
        feeder.feed();
        intake.slurp();
    }

    private void updateIntake() {
        if (xbox.getAButton()) {
            if (Robot.irBreak.get()) {
                intake.slurp();
                feeder.assist();
            } else {
                intake.stop();
                feeder.stop();
            }
            feederAssist.start();
            feederAssist.reset();
        } else if (!feederAssist.hasElapsed(0.1) && feederAssist.hasElapsed(0.00000001)) {
            //feeder.assist();
            //intake.slurp();
        } else if (xbox.getYButton()) {
            intake.spit();
            feeder.reverse();
        } else {
            intake.stop();
            feeder.stop();
        }
    }
  
}