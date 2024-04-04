package frc.robot;

import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.BasicAuto.AutoStates.*;

public class BasicAuto {
    

    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT1, ADJUST2, MOVENOTE2, TURN2, MOVEBACK2, SHOOT2, ADJUST3, MOVENOTE3, TURN3, MOVEBACK3, SHOOT3, ADJUST4, MOVENOTE4, TURN4, MOVEBACK4, SHOOT4, MOVE, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP, 750, SHOOT1, shooter::shootSpeaker);
        stateMachine.addBoolState(SHOOT1, ADJUST2, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(ADJUST2, 200, MOVENOTE2, () -> {
            swerveDrive.drive(0,0.04,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE2, 1750, TURN2, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, 0.4, 0, true, false);
        });
        stateMachine.addTimerState(TURN2, 500, MOVEBACK2, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, -0.04, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK2, 1500, SHOOT2, () -> {
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, -0.4, 0, true, false);
        });       
        stateMachine.addBoolState(SHOOT2, ADJUST3, () -> {
            swerveDrive.drive(0,-0.05,0, true, false); 
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(ADJUST3, 200, MOVENOTE3, () -> {
            swerveDrive.drive(-0.04,0.04,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE3, 2250, TURN3, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.3, 0.3, 0.2, true, false);
        });
        stateMachine.addTimerState(TURN3, 500, MOVEBACK3, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.04, -0.04, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK3, 2000, SHOOT3, () -> {
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.4, -0.3, -0.2, true, false);
        });       

        stateMachine.addBoolState(SHOOT3, ADJUST4, () -> {
            swerveDrive.drive(0,-0.05,0, true, false);
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });
        stateMachine.addTimerState(ADJUST4, 200, MOVENOTE4, () -> {
            swerveDrive.drive(0.04,0.05,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE4, 2250, TURN4, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.3, 0.3, -0.2, true, false);
        });
        stateMachine.addTimerState(TURN4, 500, MOVEBACK4, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.04, -0.04, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK4, 2250, SHOOT4, () -> {
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.4, -0.3, 0.2, true, false);
        });       

        stateMachine.addBoolState(SHOOT4, MOVE, () -> {
            swerveDrive.drive(0,-0.05,0, true, false);
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