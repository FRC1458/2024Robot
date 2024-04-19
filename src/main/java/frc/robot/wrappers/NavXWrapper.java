package frc.robot.wrappers;

import frc.robot.*;

import edu.wpi.first.wpilibj.DriverStation;

public class NavXWrapper extends Wrapper{
    private NavX navx;

    public NavXWrapper(){
        try{
            navx = NavX.getInstance();
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }
    public void operatorControl() {
        //if (isInitialized) navx.operatorControl();
    }
}