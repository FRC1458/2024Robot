package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.BasicAuto.AutoStates.*;

public class BasicAuto {

    public enum AutoStates {RESET_ENCODERS, SPIN_UP, PIVOT, SHOOT, MOVE, END}

    public static StateMachine<AutoStates> getStateMachine(Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, PIVOT, shooter::shootSpeaker);
        stateMachine.addBoolState(PIVOT, SHOOT, () -> {
            shooter.shootSpeaker();
            return shooter.pivotPointBlank();
        });
        stateMachine.addTimerState(SHOOT, 500, MOVE, () -> {
            shooter.shootSpeaker();
            feeder.feed();
        });
        stateMachine.addTimerState(MOVE, 2250, END, () -> {
            feeder.stop();
            shooter.stop();
            swerveDrive.drive(0, 0.3, 0, true);
        });
        stateMachine.addOffState(END, () -> {
            swerveDrive.drive(0, 0, 0, true);
        });
        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End

}