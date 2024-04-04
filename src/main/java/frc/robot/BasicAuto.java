package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.BasicAuto.AutoStates.*;

public class BasicAuto {

    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT, ADJUST1, MOVENOTE1, TURN, MOVEBACK1, SPIN_UP2, SHOOT2, ADJUST2, MOVENOTE2, TURN1, MOVEBACK2, SPIN_UP3, SHOOT3, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT, shooter::shootSpeaker);
        stateMachine.addBoolState(SHOOT, ADJUST1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(ADJUST1, 200, MOVENOTE1, () -> {
            swerveDrive.drive(0,0.05,0,true,true);
        });
        stateMachine.addTimerState(MOVENOTE1, 1750, TURN, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, 0.4, 0, true, false);
        });
        stateMachine.addTimerState(TURN, 500, MOVEBACK1, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, -0.05, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK1, 1750, SPIN_UP2, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, -0.4, 0, true, false);
        });       
        stateMachine.addTimerState(SPIN_UP2, 750, SHOOT2, () ->{
            swerveDrive.drive(0,0,0, true, true); 
            shooter.shootSpeaker();
        });
        stateMachine.addBoolState(SHOOT2, ADJUST2, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(ADJUST2, 200, MOVENOTE2, () -> {
            swerveDrive.drive(-0.04,0.05,0,true,true);
        });
        stateMachine.addTimerState(MOVENOTE2, 2250, TURN1, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.3, 0.3, 0.2, true, false);
        });
        stateMachine.addTimerState(TURN1, 500, MOVEBACK2, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.04, -0.05, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK2, 2250, SPIN_UP3, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.4, -0.3, -0.2, true, false);
        });       
        stateMachine.addTimerState(SPIN_UP3, 750, SHOOT3, () ->{
            swerveDrive.drive(0,0,0, true, true); 
            shooter.shootSpeaker();
        });
        stateMachine.addBoolState(SHOOT3, END, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addOffState(END, () ->{
            shooter.stop();
            feeder.stop();
            intake.stop();
            swerveDrive.drive(0,0,0, true, true); 
        });

        return stateMachine;
    }

    // Spin Up
    // Shoot
    // Move back
    // End

}