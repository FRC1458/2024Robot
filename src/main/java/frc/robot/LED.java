package frc.robot;

import static frc.robot.RobotConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int count;
    private long timer;
    
    public LED() {
        led = new AddressableLED(1);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        count = 100; //arbritrary positive number
        timer = System.currentTimeMillis();
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void rainbowPulse() {
        for(int i = ledStart; i < 53; i++) {
            ledBuffer.setHSV(i, (53 + count - i) % 180 , 255, 255);
          }
          for(int i = 53; i < 106; i++) {
            ledBuffer.setHSV(i, (count + i - 53) % 180 , 255, 255);
          }
          count++;
          led.setData(ledBuffer);
    }

    public void ChristmasStream() {
        int red = 0, green = 0,blue = 0;
        for(int i = ledStart; i < 106; i++) {
            if(i % 53 < 26) {
                red = 255;
                green = 0;
                blue = 0;
            }
            else {
                red = 0;
                green = 255;
                blue = 0;
            }

            if(timer < System.currentTimeMillis()) {
                timer = System.currentTimeMillis() + 100;
                count++;
            }
            ledBuffer.setRGB((i + count) % 108, red, green ,blue);
        }
          led.setData(ledBuffer);
    }


    public void random() {
        setSolidColor((int)(Math.random() * 50) + 100, (int)(Math.random() * 50) + 100, (int)(Math.random() * 50) + 100);
    }
    
    public void yellow() {
        setSolidColor(255, 255, 0);
    }

    public void red() {
        setSolidColor(255, 0, 0);
    }

    public void pink() {
        setSolidColor(200, 50, 50);
    }

    public void white() {
        setSolidColor(255, 255, 255);
    }

    public void orange() {
        setSolidColor(255, 50, 0);
    }

    public void green() {
        setSolidColor(0, 255, 0);
    }

    public void lightBlue() {
        setSolidColor(0, 255, 255);
    }

    public void blue() {
        setSolidColor(0, 0, 255);
    }

    public void purple() {
        setSolidColor(255,0, 255);
    }

    public void setSolidColor(int r, int g, int b) {
        for(int i = ledStart; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    
}

 