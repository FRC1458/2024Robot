package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavX{
  AHRS ahrs;

  public NavX() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  public void operatorControl() {

  }
}