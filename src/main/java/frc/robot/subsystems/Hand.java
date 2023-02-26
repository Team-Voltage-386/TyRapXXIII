package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utils.Flags;

public class Hand extends SubsystemBase {
    /*handPosition is which of the three possible possitions the hand is in
     *if handPosition = -1 it is rotated to the left
     *if handPosition = 0 it is centered and can be retracted
     *if handPosition = 1 it it rotated to the right */
    public static int handPosition = 0;
    static int pastHandPosition = 0;
    static DoubleSolenoid pcmCompressor; 
    static CANSparkMax RPickup;
    static CANSparkMax LPickup;

    public Hand()
    {
        pcmCompressor = new DoubleSolenoid(HandConstants.kDoubleSolenoidModule, PneumaticsModuleType.CTREPCM, HandConstants.kSolenoidForward, HandConstants.kSolenoidReverse);
        RPickup = new CANSparkMax(HandConstants.kRightPickupID, MotorType.kBrushless);
        LPickup = new CANSparkMax(HandConstants.kLeftPickupID, MotorType.kBrushless);
        Flags.IntakeDirection = false;
    }

    public static void ChangeMode ()
    {
        if (Flags.ConeMode)
        {
            pcmCompressor.set(Value.kForward);
        }
        else
        {
            pcmCompressor.set(Value.kReverse);
        }
    }

    public static void IntakeMotorControl(boolean intake) {
        if(intake) {
            RPickup.set(0.05);
            LPickup.set(0.05);
        }
        if(!intake) {
            RPickup.set(-0.05);
            LPickup.set(-0.05);
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
