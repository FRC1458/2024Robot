package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.LongSideAuto.AutoStates.*;

public class LongSideAuto {


    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT1, MOVE1, MOVE2, LAUNCH1, ROTATE90, LAUNCHREST, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT1, shooter::shootSpeaker);
        stateMachine.addTimerState(SHOOT1, 250, MOVE1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
        });
        stateMachine.addTimerState(MOVE1, 2000, MOVE2, () -> {
            shooter.stop();
            intake.stop();
            feeder.stop();
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(0.4, 0, -0.2, true, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(0.4, 0, 0.2, true, false);
            else swerveDrive.drive(0.4, 0, 0, true, false);
        });
        stateMachine.addTimerState(MOVE2, 4000, LAUNCH1, () -> {
            intake.slurp();
            feeder.assist();
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(0.4 * Math.cos(Math.toRadians(60)), 0.4 * Math.sin(Math.toRadians(60)), 0, true, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(0.4 * Math.cos(Math.toRadians(60)), -0.4 * Math.cos(Math.toRadians(60)), 0, true, false);
            else swerveDrive.drive(0.4, 0, 0, false, false);
        });

        stateMachine.addTimerState(LAUNCH1, 250, ROTATE90, () -> {
            intake.slurp();
            feeder.assist();
            shooter.shootAmp();
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(-0.1 * Math.cos(Math.toRadians(60)), 0.1 * Math.sin(Math.toRadians(60)), 0, true, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(-0.1 * Math.cos(Math.toRadians(60)), -0.1 * Math.sin(Math.toRadians(60)), 0, true, false);
            else swerveDrive.drive(-0.1, 0, 0, false, false);
        });
        stateMachine.addTimerState(ROTATE90, 1500, LAUNCHREST, () -> {
            shooter.stop();
            intake.stop();
            feeder.stop();
            //TEST
            if (color.equalsIgnoreCase("blue")) swerveDrive.turnToAngle(90, 0);
            else if (color.equalsIgnoreCase("red")) swerveDrive.turnToAngle(90, 0);
            else swerveDrive.stop();
        });
        stateMachine.addTimerState(LAUNCHREST, 4500, END, () -> {
            shooter.shootAmp();
            intake.slurp();
            feeder.assist();
            if (color.equalsIgnoreCase("blue")) swerveDrive.drive(-0.3, 0, 0, false, false);
            else if (color.equalsIgnoreCase("red")) swerveDrive.drive(0.3, 0, 0 ,false, false);
            else {
                intake.stop();
                feeder.stop();
                shooter.stop();
            }
        });


        stateMachine.addOffState(END, () ->{
            shooter.stop();
            feeder.stop();
            intake.stop();
            swerveDrive.stop();
        });

        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End


}