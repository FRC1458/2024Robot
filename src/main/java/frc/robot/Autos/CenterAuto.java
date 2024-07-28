package frc.robot.Autos;

import static frc.robot.Autos.CenterAuto.AutoStates.*;

import frc.robot.Feeder;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;

public class CenterAuto {
    

    public enum AutoStates {RESET_ENCODERS, SPIN_UP, SHOOT1, ADJUST2, MOVENOTE2, TURN2, MOVEBACK2, SHOOT2, ADJUST3, MOVENOTE3, TURN3, MOVEBACK3, SHOOT3, ADJUST4, MOVENOTE4, TURN4, MOVEBACK4, SHOOT4, MOVE, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color) {
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
            return true;
        });
        stateMachine.addTimerState(ADJUST2, 100, MOVENOTE2, () -> {
            swerveDrive.drive(0,0.04,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE2, 1500, TURN2, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, 0.4, 0, true, false);
        });
        stateMachine.addTimerState(TURN2, 100, MOVEBACK2, () -> {
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
            feeder.fullPow();
            intake.fullPow();
            return true;
        });
        stateMachine.addTimerState(ADJUST3, 100, MOVENOTE3, () -> {
            swerveDrive.drive(-0.04,0.04,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE3, 1850, TURN3, () -> { // 1850
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.3, 0.3, 0.18, true, false); // 0.18
        });
        stateMachine.addTimerState(TURN3, 100, MOVEBACK3, () -> {
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(-0.04, -0.04, 0, true, false);
        });
        stateMachine.addTimerState(MOVEBACK3, 1450, SHOOT3, () -> { // 1450
            shooter.shootSpeaker();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.4, -0.3, -0.18, true, false); // -0.18
        });       

        stateMachine.addBoolState(SHOOT3, ADJUST4, () -> {
            swerveDrive.drive(0,-0.05,0, true, false);
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return true;
        });
        stateMachine.addTimerState(ADJUST4, 100, MOVENOTE4, () -> {
            swerveDrive.drive(0.04,0.05,0,true,false);
        });
        stateMachine.addTimerState(MOVENOTE4, 2250, TURN4, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0.3, 0.3, -0.2, true, false);
        });
        stateMachine.addTimerState(TURN4, 100, MOVEBACK4, () -> {
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
            return true;
        });

        stateMachine.addTimerState(MOVE, 1500, END, () -> {
            swerveDrive.drive(0,0.4, 0, true, false);
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