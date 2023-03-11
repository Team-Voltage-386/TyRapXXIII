package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;
import static frc.robot.utils.Flags.*;

public class LEDSubsystem extends SubsystemBase
{
    //Intializes led length and port
    private static final int LEDPort = kLEDPort;
    private static final int LEDLength = kLEDLength;
    private double BWCycle = 0.0;

    //LEDs will alternate every LED_Speed milliseconds
    private static int LED_Speed=50;

    //Sets up the LEDs so we can write values to them
    AddressableLED led = new AddressableLED(LEDPort);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDLength);

    //Sets up all the LED values and starts the LEDs
    public LEDSubsystem() {
        led.setLength(LEDLength);
        led.setData(ledBuffer);
        led.start();
    }

    public void updateLEDS()
    {
        if (ConeMode==true)
        {
            AlternatingYellow();
        }
        else
        {
            AlternatingPurple();
        }
    }

    //Sets the single LED at index to purple
    public void setOnePurple(int index) {
        ledBuffer.setRGB(index, 208, 58, 224);
    }

    //Sets the single LED at index to yellow
    public void setOneYellow(int index) {
        ledBuffer.setRGB(index, 241, 245, 7);
    }
    
    //Sets the single LED at index to blue
    public void setOneBlue(int index) {
        ledBuffer.setRGB(index, 0, 0, 255);
    }

    //Turns the single LED at index off
    public void setOneOff(int index) {
        ledBuffer.setRGB(index, 0, 0, 0);
    }

    // public void alternateBlueYellow() {
    //     for(int i = 0; i < ledBuffer.getLength(); i++) {
    //         if(i%2 ==0) setOneBlue(i);
    //         else setOneYellow(i);
    //     }
    //     led.setData(ledBuffer);
    // }
    
    //Sets all of the LEDs on the strip to purple
    public void allPurple() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          setOnePurple(i);
        }
    }
    
    //Turns off all of the LEDs on the strip
    public void allOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          setOneOff(i);
        }
    }
    
    //Sets all of the Leds on the strip to Yellow
    public void allYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
          setOneYellow(i);
        }
    }

    //Alternates all of the LEDs between off and purple for when the Robot is in cube mode
    public void AlternatingPurple() {
        int LED_Mode=0;
        if ((Timer.getFPGATimestamp()%(LED_Speed*2))>LED_Speed)
        {
            LED_Mode=0;
        }
        else
        {
            LED_Mode=1;
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i%2==LED_Mode)
            {
                setOnePurple(i);
            }
            else
            {
                setOneOff(i);
            }
        }
    }

    //Alternates all of the LEDs between off and purple for when the Robot is in cone mode
    public void AlternatingYellow() {
        int LED_Mode=0;
        if ((Timer.getFPGATimestamp()%(LED_Speed*2))>LED_Speed)
        {
            LED_Mode=0;
        }
        else
        {
            LED_Mode=1;
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i%2==LED_Mode)
            {
                setOneYellow(i);
            }
            else
            {
                setOneOff(i);
            }
        }
    }

    //Huh? (Ask Lucas idk what he was trying to do)
    public void BlueYellow()
    {
        for(int i = 0; i < ledBuffer.getLength(); i++)
        {
            
        }
        BWCycle += 0.01;
    }
   
    // public void allWhite() {
    //     for (int i = 0; i < ledBuffer.getLength(); i++) {
    //       ledBuffer.setRGB(i, 255, 255, 255);
    //     }
    // }
    
    
    
    //   /**
    //    * inputs angle from gyro and makes the LEDs do pretty colors using a 3phase sin wave
    //    * @param a
    //    */
    //   public void setColorWithAngle(double angle) {
    //     int a = (int)angle;
    //     if(a > 360 || a < -360)  a = a%360;
    //     //logic converting degrees to radians and multiplying by 255
    //     int r = Math.abs((int)(255*Math.pow(Math.cos(2*((a*Pi)/180)), 3)));
    //     int g = Math.abs((int)(255*Math.pow(Math.cos(2*(Pi/3) + ((a*Pi)/180)), 3)));
    //     int b = Math.abs((int)(255*Math.pow(Math.cos(2*(((2*Pi)/3) + ((a*Pi)/180))), 3)));
    //     //setting RGB values
    //     for (int i = 0; i < ledBuffer.getLength(); i++) {
    //       ledBuffer.setRGB(i, r, g, b);
    //     }
    //   }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
        updateLEDS();
    }


    
}
