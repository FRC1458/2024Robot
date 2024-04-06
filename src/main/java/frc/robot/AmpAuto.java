package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;

import static frc.robot.AmpAuto.AutoStates.*;

public class AmpAuto {


    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT1, WAIT, MOVEOUT, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT1, shooter::shootSpeaker);
        stateMachine.addTimerState(SHOOT1, 500, WAIT, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
        });
        stateMachine.addTimerState(WAIT, 12000, MOVEOUT, () -> {

            swerveDrive.stop();
            shooter.stop();
            intake.stop();
            feeder.stop();
        });
        stateMachine.addTimerState(MOVEOUT, 1500, END, () -> {
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(0.4, 0.4 * (Math.sqrt(3) / 3.0), -0.3, true, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(-0.4, 0.4 * (Math.sqrt(3) / 3.0), 0.3, true, false);
            else swerveDrive.drive(0.4 * Math.sqrt(0.5), 0.4 * Math.sqrt(0.5), 0, true, false);
        });
        stateMachine.addOffState(END, () ->{
            swerveDrive.stop();
        });

        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End

}