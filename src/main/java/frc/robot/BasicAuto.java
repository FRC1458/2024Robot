package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.BasicAuto.AutoStates.*;

public class BasicAuto {

    public enum AutoStates {SPIN_UP, SHOOT, MOVE, END}

    public static StateMachine<AutoStates> getStateMachine(Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(SPIN_UP);
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT, shooter::shootSpeaker);
        stateMachine.addTimerState(SHOOT, 500, MOVE, () -> {
            shooter.shootSpeaker();
            feeder.feed();
        });
        stateMachine.addTimerState(MOVE, 1000, END, () -> {
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