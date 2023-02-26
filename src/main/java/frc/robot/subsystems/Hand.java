package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;
import frc.robot.utils.Flags;

public class Hand extends SubsystemBase {
    //The talon motor is ID 2

    /*handPosition is which of the three possible possitions the hand is in
     *if handPosition = -1 it is rotated to the left
     *if handPosition = 0 it is centered and can be retracted
     *if handPosition = 1 it it rotated to the right */
    public static int handPosition = 0;
    static int pastHandPosition = 0;

    static DoubleSolenoid pcmCompressor; 
    static TalonSRX RotationalMotor;

    public Hand()
    {
        pcmCompressor = new DoubleSolenoid(HandConstants.kDoubleSolenoidModule, PneumaticsModuleType.CTREPCM, HandConstants.kSolenoidForward, HandConstants.kSolenoidReverse);
        RotationalMotor = new TalonSRX(HandConstants.kArmRotator);
        
    }

    public void Positioning()
    {
        System.out.println(RotationalMotor.getSelectedSensorPosition());
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

        }
        if (handPosition == 0 && pastHandPosition == 1);
        {
            pastHandPosition = handPosition;
        }
        if (handPosition == 0 && pastHandPosition == -1);
        {
            pastHandPosition = handPosition;
        }
        if (handPosition == -1);
        {
            pastHandPosition = handPosition;
        }  
    }


}
