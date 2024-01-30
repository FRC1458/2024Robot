package frc.robot;

public interface Controller {

     boolean fieldOriented();

     double getSwerveR();
     double getSwerveX();
     double getSwerveY();

     boolean resetNavXButton();
     boolean getButtonX();
     boolean getButtonA();

}
