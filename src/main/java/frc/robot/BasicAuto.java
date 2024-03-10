package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.BasicAuto.AutoStates.*;

public class BasicAuto {

    public enum StartPosition {LEFT, CENTER, RIGHT}
    public enum AutoStates {SPIN_UP, PIVOT, SHOOT, MOVE, SPIN4JEREMY, PREP, END}
    public final static StartPosition startPosition = StartPosition.LEFT;

    public static StateMachine<AutoStates> getStateMachine(Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(SPIN_UP);
        stateMachine.addTimerState(SPIN_UP, 1000, PIVOT, shooter::shootSpeaker);
        stateMachine.addBoolState(PIVOT, SHOOT, () -> {
            shooter.shootSpeaker();
            return shooter.pivotPointBlank();
        });
        stateMachine.addTimerState(SHOOT, 500, MOVE, () -> {
            shooter.shootSpeaker();
            feeder.feed();
        });
        stateMachine.addTimerState(MOVE, 2500, SPIN4JEREMY, () -> moveBack(feeder, shooter, swerveDrive));
        stateMachine.addBoolState(SPIN4JEREMY, PREP, () -> spin4Jeremy(swerveDrive));
        stateMachine.addBoolState(PREP, END, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            swerveDrive.drive(0, 0, 0, true);
            return true;
        });
        stateMachine.addOffState(END, () -> {});
        return stateMachine;
    }

    public static void moveBack(Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        feeder.stop();
        shooter.stop();
        switch (startPosition) {
            case LEFT -> swerveDrive.drive(-0.2598, 0.15, 0, true);
            case CENTER -> swerveDrive.drive(0, 0.3, 0, true);
            case RIGHT -> swerveDrive.drive(0.2598, 0.15, 0, true);
            default -> {}
        }
    }

    public static boolean spin4Jeremy(SwerveDrive swerveDrive) {
        return switch (startPosition) {
            case LEFT -> swerveDrive.turnToAngle(150);
            case CENTER -> swerveDrive.turnToAngle(180);
            case RIGHT -> swerveDrive.turnToAngle(210);
            default -> true;
        };
    }

    // Spin Up
    // Shoot
    // Move Back
    // Spin for Jeremy
    // End

}