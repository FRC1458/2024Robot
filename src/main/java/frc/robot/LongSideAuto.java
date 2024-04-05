package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.LongSideAuto.AutoStates.*;

public class LongSideAuto {


    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT1, MOVE1, MOVE2, LAUNCH1, ROTATE90, LAUNCHREST, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT1, shooter::shootSpeaker);
        stateMachine.addBoolState(SHOOT1, MOVE1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(MOVE1, 1000, MOVE2, () -> {
            shooter.stop();
            intake.stop();
            feeder.stop();
            swerveDrive.drive(0.4, 0, 0, true, false);
        });
        stateMachine.addTimerState(MOVEY, 4000, TURN2, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.1, 0.4, 0, true, false);
        });

        stateMachine.addTimerState(TURN2, 100, MOVEBACKX, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.04, -0.04, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACKX, 1500, MOVEBACKY, () -> {
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.1, -0.4, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACKY, 1500, SHOOT2, () -> {
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.3, 0, 0, true, false);
        });
        stateMachine.addBoolState(SHOOT2, MOVE, () -> {
            swerveDrive.drive(0,-0,0, true, false);
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addTimerState(MOVE, 1500, END, () -> {
            swerveDrive.drive(0,0.4, 0, true, false);
        });

        stateMachine.addOffState(END, () ->{
            shooter.stop();
            feeder.stop();
            intake.stop();
            swerveDrive.drive(0,0,0, true, false);
        });

        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End

}