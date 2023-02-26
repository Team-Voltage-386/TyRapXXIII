package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import frc.robot.utils.Flags;

public class Hand extends SubsystemBase {
    /*handPosition is which of the three possible possitions the hand is in
     *if handPosition = -1 it is rotated to the left
     *if handPosition = 0 it is centered and can be retracted
     *if handPosition = 1 it it rotated to the right */
    public static int handPosition = 0;
    static int pastHandPosition = 0;

    //Declaration of motors and pnumatics
    static DoubleSolenoid pcmCompressor; 
    static TalonSRX HandRotationalMotor;

    //Limit declaration
    private DigitalInput HandLimitSwitch; // LIMIT READING TRUE MEANS SWTICH NOT HIT
    public boolean HandHitLimit; // HandHitLimit WILL READ TRUE WHEN IT HITS THE LIMIT

    public Hand()
    {
        pcmCompressor = new DoubleSolenoid(HandConstants.kDoubleSolenoidModule, PneumaticsModuleType.CTREPCM, HandConstants.kSolenoidForward, HandConstants.kSolenoidReverse);
        HandRotationalMotor = new TalonSRX(HandConstants.kHandRotator);
        HandLimitSwitch = new DigitalInput(HandConstants.kHandLimitSwitch);
        HandRotationalMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        
    }

    public void Positioning()
    {
        System.out.println(HandRotationalMotor.getSelectedSensorPosition());
    }

    public static void ChangeMode ()
    {
        if (Flags.ConeMode)
        {
            pcmCompressor.set(Value.kForward);
        }
        if (!Flags.ConeMode)
        {
            pcmCompressor.set(Value.kReverse);
        }
    }

    public static boolean canRetract()
    {
        if (handPosition==0)
        {
            return true;
        }
        return false;
    }

    public void RotateHand()
    {
        if (handPosition == 1);
        {
            pastHandPosition = handPosition;
            while (!getHandLimitSwitch())
            {
                //code to move
            }
        }
        if (handPosition == 0 && pastHandPosition == 1);
        {
            pastHandPosition = handPosition;
            while (!getHandLimitSwitch())
            {
                //code to move
            }
        }
        if (handPosition == 0 && pastHandPosition == -1);
        {
            pastHandPosition = handPosition;
            while (!getHandLimitSwitch())
            {
                //code to move
            }
        }
        if (handPosition == -1);
        {
            pastHandPosition = handPosition;
            while (!getHandLimitSwitch())
            {
                //code to move
            }
        }  
    }

    //Returns true when the limit switch hits its limit
    public boolean getHandLimitSwitch()
    {
        return !HandLimitSwitch.get();
    }

    public void setRotation(double power)
    {
        HandRotationalMotor.set(ControlMode.PercentOutput, power);
    }

    public void setLimitClear()
    {
        System.out.println(getHandLimitSwitch());
    }
}
