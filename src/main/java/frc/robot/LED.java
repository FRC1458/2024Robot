package frc.robot;

import static frc.robot.RobotConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int count;
    
    public LED() {
        led = new AddressableLED(1);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        count = 100; //arbritrary positive number
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void autoLights() {
        for(int i = ledStart; i < 53; i++) {
            ledBuffer.setHSV(i, (53 + count - i) % 180 , 255, 255);
          }
          for(int i = 53; i < 106; i++) {
            ledBuffer.setHSV(i, (count + i - 53) % 180 , 255, 255);
          }
          count++;
          led.setData(ledBuffer);
    }

    public void intakeActiveLights() {
        for(int i = 18; i < 120;i++) {
            ledBuffer.setRGB(i, 255 , 255, 0);
        }
        led.setData(ledBuffer);  
    }

    public void intakeStallLights() {
        for(int i = 18; i < 120;i++) {
            ledBuffer.setRGB(i, 255 , 0, 0);
        }
        led.setData(ledBuffer);  
    }
    
    public void intakePanicLights() {
        for(int i = 18; i < 120;i++) {
            ledBuffer.setRGB(i, 255 , 0, 0);
        }
        led.setData(ledBuffer);  
    }

    public void intakeTempLights() {
        for(int i = 18; i < 120; i++) {
            ledBuffer.setRGB(i, 200, 50, 50); 
        }
        led.setData(ledBuffer); 
    }

    public void sourceLights() {
        for(int i = 18; i < 120; i++) {
            ledBuffer.setRGB(i, 200, 200, 200);
            led.setData(ledBuffer);
        }
    }

    public void teleopLights() {
        for(int i = 18; i < 120;i++) {
            ledBuffer.setRGB(i, 255 , 255, 255);
        }
        led.setData(ledBuffer);  
    }

    public void disabledLights() {
        for (int i = 18; i < 120; i++) {
            ledBuffer.setRGB(i, 255, 80, 0);
        }
        led.setData(ledBuffer);
    }

    public void noteDetectedLights() {
        for(int i = 18; i < ledBuffer.getLength();i++) {
            ledBuffer.setRGB(i, 0 , 255, 0);
        }
        led.setData(ledBuffer);
    }

    public void rampedUpLights() {
        for(int i = 18; i < ledBuffer.getLength();i++) {
            ledBuffer.setRGB(i, 0 , 255, 255);
        }
        led.setData(ledBuffer);
    }

    public void intakeOverrideLights() {
        for(int i = 18; i < ledBuffer.getLength();i++) {
            ledBuffer.setRGB(i, 255, 0, 255);
        }
        led.setData(ledBuffer);
    }

    public void feedNoteDetectedLights() {
            if(count < 100) {
                intakeActiveLights();
            }
            else if(count < 200) {
                noteDetectedLights();
            }
            else{
                count = 0;
            }
    }
    
    public void feedRevLights() {
            if(count < 100) {
                intakeActiveLights();
            }
            else if(count < 200) {
                rampedUpLights();
            }
            else{
                count = 0;
            }
    }
}

 