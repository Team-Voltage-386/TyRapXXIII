package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import static frc.robot.Constants.HandConstants.*;
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
        Flags.ConeMode = true;
        // RPickup.setSmartCurrentLimit(1,2);
        // LPickup.setSmartCurrentLimit(1,2);
    }

    public static void ChangeMode ()
    {
        if (Flags.ConeMode)
        {
            pcmCompressor.set(Value.kForward);
            RPickup.set(kConeIntakeSpeed);
            LPickup.set(kConeIntakeSpeed);
            Flags.ConeMode = false;
        }
        else
        {
            pcmCompressor.set(Value.kReverse);
            RPickup.set(kCubeIntakeSpeed);
            LPickup.set(kCubeIntakeSpeed);
            Flags.ConeMode = true;
        }
    }

    public static void IntakeMotorControl(boolean intake) {
        if(!Flags.ConeMode) {
            if(intake) {
                RPickup.set(kConeIntakeSpeed);
                LPickup.set(kConeIntakeSpeed);
            }
            if(!intake) {
                RPickup.set(-kConeIntakeSpeed);
                LPickup.set(-kConeIntakeSpeed);
            }
        }
        else {
            if(intake) {
                RPickup.set(kCubeIntakeSpeed);
                LPickup.set(kCubeIntakeSpeed);
            }
            if(!intake) {
                RPickup.set(-kCubeIntakeSpeed);
                LPickup.set(-kCubeIntakeSpeed);
            }
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

    @Override
    public void periodic() {
        Mode.setBoolean(Flags.ConeMode);
        CurrentR.setDouble(RPickup.getOutputCurrent());
        CurrentL.setDouble(LPickup.getOutputCurrent());
    }

    private static final ShuffleboardTab HandTab = Shuffleboard.getTab("Hand Tab");
    private static final GenericEntry Mode = HandTab.add("ConeMode", true).getEntry();
    private static final GenericEntry CurrentR = HandTab.add("CurrentR", 0.0).getEntry();
    private static final GenericEntry CurrentL = HandTab.add("CurrentL", 0.0).getEntry();
}
