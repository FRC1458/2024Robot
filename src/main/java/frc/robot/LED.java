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
        led.setData(ledBuffer);
        led.setLength(ledLength);
        led.start();
    }

    public void autoLights() {
        for(int i = 18; i < ledBuffer.getLength();i++) {
            ledBuffer.setRGB(i, 255 , 0, 0);
        }
        led.setData(ledBuffer);
    }

    public void teleopLights() {
        for(int i = ledStart; i < 53; i++) {
            ledBuffer.setHSV(i, (53 + count - i) % 180 , 255, 255);
          }
          for(int i = 53; i < 106; i++) {
            ledBuffer.setHSV(i, (count + i - 53) % 180 , 255, 255);
          }
          count++;
          led.setData(ledBuffer);
          
    }

    public void disabledLights() {
        for (int i = 18; i < ledBuffer.getLength(); i++) {
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

}
