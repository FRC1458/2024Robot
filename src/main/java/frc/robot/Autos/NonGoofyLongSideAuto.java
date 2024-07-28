package frc.robot.Autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Feeder;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Trajectory.PathPlannerTraj;
import frc.robot.Trajectory.Trajectory;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.Autos.NonGoofyLongSideAuto.AutoStates.*;

public class NonGoofyLongSideAuto {

    public enum AutoStates {RESET_ENCODERS, SHOOT0, SPINUP1, IN1, OUT1, SHOOT1, IN2, OUT2, SHOOT2, IN3, OUT3, SHOOT3, IN4, OUT4, SHOOT4, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color, DigitalInput irBreak) {

        List<Trajectory> trajectories = new ArrayList<>();
        trajectories.add(new PathPlannerTraj("SourceInN7", swerveDrive));
        trajectories.add(new PathPlannerTraj("SourceOutN7", swerveDrive));
        trajectories.add(new PathPlannerTraj("SourceInN8", swerveDrive));
        trajectories.add(new PathPlannerTraj("SourceOutN8", swerveDrive));


        //use trajectories.add(new ChoreoPathplannerTraj("FILENAME", swerveDrive));

        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SPINUP1, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });

        stateMachine.addTimerState(SPINUP1, 500, SHOOT0, shooter::shootSpeaker);
        stateMachine.addBoolState(SHOOT0, IN1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return irBreak.get();
        });

        stateMachine.addBoolState(IN1, OUT1, (ts) -> {
            if (ts > 900 && irBreak.get()) {
                feeder.feed();
                intake.slurp();
            } else {
                feeder.stop();
                intake.stop();
            }
            shooter.stop();
            return trajectories.get(0).sample(ts);
        });
        stateMachine.addBoolState(OUT1, SHOOT1, (ts) -> {
            feeder.stop();
            intake.stop();
            shooter.shootSpeaker();
            return trajectories.get(1).sample(ts);
        });
        stateMachine.addBoolState(SHOOT1, IN2, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return irBreak.get();
        });

        stateMachine.addBoolState(IN2, OUT2, (ts) -> {
            if (ts > 1250 && irBreak.get()) {
                feeder.feed();
                intake.slurp();
            } else {
                feeder.stop();
                intake.stop();
            }
            shooter.stop();
            return trajectories.get(2).sample(ts);
        });
        stateMachine.addBoolState(OUT2, SHOOT2, (ts) -> {
            feeder.stop();
            intake.stop();
            shooter.shootSpeaker();
            return trajectories.get(3).sample(ts);
        });
        stateMachine.addBoolState(SHOOT2, END, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return irBreak.get();
        });

        stateMachine.addOffState(END, () -> {
            shooter.stop();
            feeder.stop();
            intake.stop();
            swerveDrive.stop();
        });
        return stateMachine;

    }

}