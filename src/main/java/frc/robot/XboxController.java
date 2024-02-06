package frc.robot;


import frc.robot.wrappers.XboxControllerWrapper;

public class XboxController implements Controller {


    XboxControllerWrapper xbox;
    public XboxController () {
        xbox = new XboxControllerWrapper(0);
    }

    @Override
    public boolean resetNavXButton() {
        return xbox.getStartButton();
    }

    @Override
    public boolean fieldOriented() {
        return true;
    }

    @Override
    public double getSwerveR() {
        return xbox.getRightX();
    }

    @Override
    public double getSwerveX() {
        return xbox.getLeftX();
    }

    @Override
    public double getSwerveY() {
        return xbox.getLeftY();
    }

    @Override
    public boolean getButtonX() {
        return xbox.getXButton();
    }

    @Override
    public boolean getButtonA() {
        return xbox.getAButton();
    }

    @Override
    public boolean getRightTrigger() {
        return (xbox.getRightTriggerAxis() > 0.7);
    }
    
    public boolean getLeftTrigger() {
        return (xbox.getLeftTriggerAxis() > 0.7);
    }

}
