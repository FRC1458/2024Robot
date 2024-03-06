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

    public enum ShootState {ANGLE, SPIN_UP, SHOOT, OFF}

    private final XboxController xbox;
    private final StateMachine<ShootState> stateMachine = new StateMachine<>(ShootState.SPIN_UP);

    public IFSAuto(Intake intake, Feeder feeder, Shooter shooter, XboxController xbox) {

        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.xbox = xbox;

        initStateMachine();

    }

    public void initStateMachine() {
        stateMachine.addTimerState(SPIN_UP, 750, ANGLE, this::spinUp);
        stateMachine.addBoolState(ANGLE, SHOOT, shooter::pivotToSpeaker);
        stateMachine.addTimerState(SHOOT, 500, OFF, this::shoot);
        stateMachine.addOffState(OFF, shooter::stop);
    }

    @Override
    public void update() {
        updateIntake();
        updatePivot();
        updateShooter();
    }

    public void updateShooter() {
        if (xbox.getRightBumperPressed() || xbox.getLeftBumperPressed()) stateMachine.reset();
        if (xbox.getRightBumper() || xbox.getLeftBumper()) stateMachine.run();
        else shooter.stop();
    }

    private void spinUp() {
        if (xbox.getRightBumper()) shooter.shootSpeaker();
        else shooter.shootAmp();
        shooter.pivotToSpeaker();
    }

    private void shoot() {
        spinUp();
        feeder.feed();
        intake.slurp();
    }

    private void updateIntake() {
        if (xbox.getAButton()) {
            intake.slurp();
            feeder.stop();
            feederAssist.reset();
        } else if (!feederAssist.hasElapsed(0.25)) {
            feeder.assist();
        } else if (xbox.getYButton()) {
            intake.spit();
            feeder.reverse();
        } else {
            intake.stop();
            feeder.stop();
        }
    }

    private void updatePivot() {
        shooter.displayPivot();
        if(xbox.getPOV() == 0) {
            shooter.moveUp();
        } else if(xbox.getPOV() == 180) {
            shooter.moveDown();
        } else {
            shooter.stopPivot();
        }
    }
  
}