package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX {

  private static final AHRS navx = new AHRS(SPI.Port.kMXP);
  private Rotation2d rotationOffset;

  private NavX() {
    rotationOffset = new Rotation2d();
    SmartDashboard.putData("NavX", navx);
  }

  public static NavX getInstance() {
    return new NavX();
  }

  public void reset() {
    navx.reset();
  }

  public void reset(Rotation2d rotationOffset) {
    this.rotationOffset = rotationOffset;
    navx.reset();
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d().minus(rotationOffset);
  }

}