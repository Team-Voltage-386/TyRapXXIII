package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {

    private static final int LEDPort = kLEDPort;
    private static final int LEDLength = kLEDLength;
    private double BWCycle = 0.0;

    AddressableLED led = new AddressableLED(LEDPort);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLength);

    public LEDSubsystem() {
        led.setLength(LEDLength);
        led.setData(ledBuffer);
        led.start();
    }

    public void setOnePurple(int index) {
        ledBuffer.setRGB(index, 208, 58, 224);
    }

    public void setOneYellow(int index) {
        ledBuffer.setRGB(index, 255, 100, 0);
    }

    public void setOneBlue(int index) {
        ledBuffer.setRGB(index, 0, 0, 200);
    }

    public void alternateBlueYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 2 == 0)
                setOneYellow(i);
            else
                setOneBlue(i);
        }
        led.setData(ledBuffer);
    }

    double BYCycle = 0;

    public void patternBlueYellow() {
        BYCycle += 0.1;
        int a = Math.abs((int) (5 * Math.cos(BYCycle))) + 1;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % a == 0) {
                setOneBlue(i);
            } else {
                setOneYellow(i);
            }
        }
    }

    public void allPurple() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setOnePurple(i);
        }
    }

    public void allBlue() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setOneBlue(i);
        }
    }

    public void allOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void allYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setOneYellow(i);
        }
    }

    // public void allWhite() {
    // for (int i = 0; i < ledBuffer.getLength(); i++) {
    // ledBuffer.setRGB(i, 255, 255, 255);
    // }
    // }

    // /**
    // * inputs angle from gyro and makes the LEDs do pretty colors using a 3phase
    // sin wave
    // * @param a
    // */
    // public void setColorWithAngle(double angle) {
    // int a = (int)angle;
    // if(a > 360 || a < -360) a = a%360;
    // //logic converting degrees to radians and multiplying by 255
    // int r = Math.abs((int)(255*Math.pow(Math.cos(2*((a*Pi)/180)), 3)));
    // int g = Math.abs((int)(255*Math.pow(Math.cos(2*(Pi/3) + ((a*Pi)/180)), 3)));
    // int b = Math.abs((int)(255*Math.pow(Math.cos(2*(((2*Pi)/3) + ((a*Pi)/180))),
    // 3)));
    // //setting RGB values
    // for (int i = 0; i < ledBuffer.getLength(); i++) {
    // ledBuffer.setRGB(i, r, g, b);
    // }
    // }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

}
