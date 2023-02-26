package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import static frc.robot.utils.Flags.*;

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

    @Override
    public void periodic() 
    {
        updateWidgets();
        System.out.println(getPositioning());
    }


    public double getPositioning()
    {
        return (HandRotationalMotor.getSelectedSensorPosition());
    }

    public static void ChangeMode ()
    {
        if (ConeMode)
        {
            pcmCompressor.set(Value.kForward);
        }
        if (!ConeMode)
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

    public void setLimitClear()
    {
        System.out.println(getHandLimitSwitch());
    }

    ShuffleboardTab HandTab = Shuffleboard.getTab("Hand Variables");

    private GenericPublisher HandWidget = HandTab.add("Hand Position", 0.0).withPosition(0, 0).withSize(1, 1) .getEntry();
    private GenericPublisher HandLimitWidget = HandTab.add("Magnetic limit", false).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#009900")).withPosition(0, 1).withSize(1, 1).getEntry();


    private void updateWidgets()
    {
        HandWidget.setDouble(getPositioning());
        HandLimitWidget.setBoolean(getHandLimitSwitch());
    }
}
