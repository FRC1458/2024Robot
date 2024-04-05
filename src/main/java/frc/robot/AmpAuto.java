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
        stateMachine.addBoolState(SHOOT1, WAIT, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(WAIT, 13500, MOVEOUT, () -> {
            swerveDrive.drive(0,0,0,true,false);
            shooter.stop();
            intake.stop();
            feeder.stop();
        });
        stateMachine.addTimerState(MOVEOUT, 1500, END, () -> {
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(-0.4 * Math.cos(Math.toRadians(60)), 0.4 * Math.sin(Math.toRadians(60)), -0.3, true, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(0.4 * Math.cos(Math.toRadians(60)), 0.4 * Math.sin(Math.toRadians(60)), 0.3, true, false);
            else swerveDrive.drive(Math.sqrt(0.5), Math.sqrt(0.5), 0, true, false);
        });
        stateMachine.addOffState(END, () ->{
            swerveDrive.drive(0,0,0, true, false);
        });

        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End

}