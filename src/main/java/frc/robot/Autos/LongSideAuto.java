package frc.robot.Autos;

import frc.robot.Feeder;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;

import static frc.robot.Autos.LongSideAuto.AutoStates.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class LongSideAuto {


    public enum AutoStates {RESET_ENCODERS, SPIN_UP1, SHOOT1, MOVE1, ROT1, MOVE2, MOVE3, ROT2, MOVE4, SPIN_UP2, SHOOT2, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color) {
        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPIN_UP1, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addTimerState(SPIN_UP1, 750, SHOOT1, shooter::shootSpeaker);
        stateMachine.addTimerState(SHOOT1, 250, MOVE1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
        });



        stateMachine.addTimerState(MOVE1, 1500, ROT1, () -> {
            shooter.stop();
            intake.stop();
            feeder.stop();
            swerveDrive.drive(0, 0.4, 0, true, false);
        });
        stateMachine.addBoolState(ROT1, MOVE2, () -> {
            double err = Rotation2d.fromDegrees((color.equalsIgnoreCase("red")) ? 60 : -60).minus(swerveDrive.navxAngle()).getDegrees();
            if (Math.abs(err) < 5) return true;
            swerveDrive.driveRaw(0, 0, -0.008 * err, true, false);
            return false;
        });
        stateMachine.addTimerState(MOVE2, 4000, MOVE3, () -> {
            shooter.stop();
            intake.slurp();
            feeder.assist();
            swerveDrive.drive(0, 0.25, 0, false, false);
        });
        stateMachine.addTimerState(MOVE3, 4000, ROT2, () -> {
            intake.stop();
            feeder.stop();
            swerveDrive.drive(0, -0.25, 0, false, false);
        });
        stateMachine.addBoolState(ROT2, MOVE4, () -> {
            double err = Rotation2d.fromDegrees(0).minus(swerveDrive.navxAngle()).getDegrees();
            if (Math.abs(err) < 5) return true;
            swerveDrive.driveRaw(0, 0, -0.008 * err, true, false);
            return false;
        });
        stateMachine.addTimerState(MOVE4, 2000, SPIN_UP2, () -> {
            swerveDrive.drive(0, 0.4, 0, true, false);
        });



        stateMachine.addTimerState(SPIN_UP2, 750, SHOOT2, shooter::shootSpeaker);
        stateMachine.addTimerState(SHOOT2, 250, END, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
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