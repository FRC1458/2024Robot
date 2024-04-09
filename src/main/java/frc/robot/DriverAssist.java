package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.swervedrive.SwerveDrive;

public class DriverAssist {
   //commented code is waiting on some other code to be pushed
    //Limelight intakeLimelight = new Limelight(0);
    //Limelight shooterLimelight = new Limelight (1);

    SwerveDrive swerveDrive;
    PIDController intakePID;
    public DriverAssist(SwerveDrive swerve) {
        swerveDrive = swerve;
        intakePID = new PIDController(0.01, 0, 0); //test pid obviously, pid will work if we can tune it
    }


    //TO DO: xbox1 will not control bot while running this (but keep a backup way to cancel this from xbox1 just in case)
    //xbox2 will control this, a button must be held for this code to run
    public boolean intakeAssist() {
        /*if (intakeAssist.getTarget()) { //write public boolean getTarget() {return table.getEntry("tv")} from Limelight.java
        if (Math.abs(intakeAssist.tx()) < 0.5 && Math.abs(intakeAssist.ty()) < 0.5) { arbitrary numbers, also make a tx and ty method returning tx and ty
        if (Robot.noteDetected()) return true;
        else if (Math.abs(intakeAssist.ty()) >= 0.5) swerveDrive.drive(0, 0.5, 0, false, false);
        else swerveDrive.drive(intakePID.calculate(intakeLimelight.tx()), intakePID.calculate(Math.abs(intakeLimelight.ty())), 0, false, false);
        }
        }
         */
        //else return false;
    }

    public boolean speakerAssist() {
        //Get position from shooterLimelight and use botpose/megapose/whatever to get a good reading,
        //only update values here if they are above some (noise) threshold
        //Use values to determine which angle of speaker to shoot from
        //PID or just go in speaker direction for movement
        //Calculate velocity and acceleration to constantly predict when we reach speaker
        //Use prediction to start ramp up early
        //Start indexer and intake BEFORE in range, timed so that as the note flies out we make it in range
        //Try and see how quick we can get it, but PRIORITIZE CONSISTENCY
    }
}
