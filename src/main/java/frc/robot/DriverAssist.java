package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class DriverAssist {
   //commented code is waiting on some other code to be pushed
    Limelight intakeLimelight = new Limelight(0);
    //Limelight shooterLimelight = new Limelight (1);
    ArrayList<Double> txList = new ArrayList<Double>();
    double txSum;

    SwerveDrive swerveDrive;
    Intake intake;
    public DriverAssist(SwerveDrive swerve, Intake intake) {
        swerveDrive = swerve;
        this.intake = intake;
    }


    //TO DO: xbox1 will not control bot while running this (but keep a backup way to cancel this from xbox1 just in case)
    //xbox2 will control this, a button must be held for this code to run
    public void intakeAssist(double x, double y, double r) {
        if (intakeLimelight.getTarget()) {
           txList.add(intakeLimelight.tx());
           txSum += intakeLimelight.tx();;
           //for loop is probably a bad idea
           if (txList.size() > 10) {
               txSum -= txList.get(0);

               txList.remove(0);
           }

            if (Math.abs(intakeLimelight.tx()) > 0.5) {
                swerveDrive.driveRaw(x, y, Math.min(1.0, (txSum / txList.size()) * 0.05), true, false);
            }
            swerveDrive.driveRaw(0, 0.3, 0, false, false);
            intake.slurp();
        }
    }

    /*public boolean speakerAssist() {
        //Get position from shooterLimelight and use botpose/megapose/whatever to get a good reading,
        //only update values here if they are above some (noise) threshold
        //Use values to determine which angle of speaker to shoot from
        //PID or just go in speaker direction for movement
        //Calculate velocity and acceleration to constantly predict when we reach speaker
        //Use prediction to start ramp up early
        //Start indexer and intake BEFORE in range, timed so that as the note flies out we make it in range
        //Try and see how quick we can get it, but PRIORITIZE CONSISTENCY
    }*/
}
