package frc.robot;

public interface Controller {

     boolean fieldOriented();

     double getSwerveR();
     double getSwerveX();
     double getSwerveY();

     boolean resetNavXButton();
     boolean getButtonX();
     boolean getButtonAPressed();
     boolean getRightTrigger();
     boolean getLeftTrigger();
}
