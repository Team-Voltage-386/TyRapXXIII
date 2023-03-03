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
import static frc.robot.Constants.HandConstants.*;
import static frc.robot.utils.Flags.*;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants.*;
import frc.robot.utils.Flags;

public class Hand extends SubsystemBase {
    /*
     * handPosition is which of the three possible possitions the hand is in
     * if handPosition = -1 it is rotated counter-clockwise
     * if handPosition = 0 it is centered and can be retracted
     * if handPosition = 1 it it rotated clockwise
     */

    public int handPosition;

    boolean handTurningClockwise;
    double targHandPos = 0;

    // Declaration of motors and pnumatics
    public DoubleSolenoid pcmCompressor;
    public TalonSRX HandRotationalMotor;

    // Limit declaration
    private DigitalInput HandLimitSwitch; // LIMIT READING TRUE MEANS SWTICH NOT HIT
    public boolean HandHitLimit; // HandHitLimit WILL READ TRUE WHEN IT HITS THE LIMIT

    public Hand() {
        pcmCompressor = new DoubleSolenoid(kDoubleSolenoidModule, PneumaticsModuleType.CTREPCM, kSolenoidForward,
                kSolenoidReverse);
        HandRotationalMotor = new TalonSRX(kHandRotator);
        HandLimitSwitch = new DigitalInput(kHandLimitSwitch);
        HandRotationalMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        handPosition = 0;

        RPickup = new CANSparkMax(HandConstants.kRightPickupID, MotorType.kBrushless);
        LPickup = new CANSparkMax(HandConstants.kLeftPickupID, MotorType.kBrushless);
        Flags.IntakeDirection = false;
        Flags.ConeMode = true;
    }

    @Override
    public void periodic() {
        setHandMotor();
        updateWidgets();
    }

    public double getPositioning() {
        return (HandRotationalMotor.getSelectedSensorPosition());
    }

    public CANSparkMax RPickup;
    public CANSparkMax LPickup;
    private static final ShuffleboardTab HandTab = Shuffleboard.getTab("Hand Tab");
    private final GenericEntry Mode = HandTab.add("ConeMode", true).getEntry();
    private final GenericEntry CurrentR = HandTab.add("CurrentR", 0.0).getEntry();
    private final GenericEntry CurrentL = HandTab.add("CurrentL", 0.0).getEntry();

    public void ChangeMode() {
        if (ConeMode) {
            pcmCompressor.set(Value.kForward);
        }
        if (!ConeMode) {
            RPickup.setSmartCurrentLimit(35, 15);
            LPickup.setSmartCurrentLimit(35, 15);
            pcmCompressor.set(Value.kForward);
            RPickup.set(kConeIntakeSpeed);
            LPickup.set(kConeIntakeSpeed);
            Flags.ConeMode = false;

        } else {
            RPickup.setSmartCurrentLimit(5, 10);
            LPickup.setSmartCurrentLimit(5, 10);
            pcmCompressor.set(Value.kReverse);
            RPickup.set(kCubeIntakeSpeed);
            LPickup.set(kCubeIntakeSpeed);
            Flags.ConeMode = true;
        }
    }

    public void IntakeMotorControl(boolean intake) {
        if (!Flags.ConeMode) {
            if (intake) {
                RPickup.set(kConeIntakeSpeed);
                LPickup.set(kConeIntakeSpeed);
            }
            if (!intake) {
                RPickup.set(-kConeIntakeSpeed);
                LPickup.set(-kConeIntakeSpeed);
            }
        } else {
            if (intake) {
                RPickup.set(kCubeIntakeSpeed);
                LPickup.set(kCubeIntakeSpeed);
            }
            if (!intake) {
                RPickup.set(-kCubeIntakeSpeed);
                LPickup.set(-kCubeIntakeSpeed);
            }
        }
    }

    public boolean canRetract() {
        if (handPosition == 0) {
            return true;
        }
        return false;
    }

    private void setHandMotor() {
        if (canRotate) {
            if (handPosition == 1 && !getHandLimitSwitch() && getPositioning() < 75) {
                HandRotationalMotor.set(ControlMode.PercentOutput, -kRotationSpeed);
            } else if (handPosition == 0 && handTurningClockwise && !getHandLimitSwitch() && getPositioning() > 0) {
                HandRotationalMotor.set(ControlMode.PercentOutput, kRotationSpeed);
            } else if (handPosition == 0 && !handTurningClockwise && !getHandLimitSwitch() && getPositioning() < 0) {
                HandRotationalMotor.set(ControlMode.PercentOutput, -kRotationSpeed);
            } else if (handPosition == -1 && !getHandLimitSwitch() && getPositioning() > -75) {
                HandRotationalMotor.set(ControlMode.PercentOutput, kRotationSpeed);
            } else {
                HandRotationalMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            HandRotationalMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }

    public void RotateHand(boolean isRightBumper) {
        if (isRightBumper) {
            handPosition++;
            handTurningClockwise = false;
        } else {
            handPosition--;
            handTurningClockwise = true;
        }
        if (handPosition > 1 || handPosition < -1) {
            if (handPosition > 1) {
                handPosition = 1;
            }
            if (handPosition < -1) {
                handPosition = -1;
            }
        }
    }

    // Returns true when the limit switch hits its limit
    public boolean getHandLimitSwitch() {
        return !HandLimitSwitch.get();
    }

    public void setLimitClear() {

    }

    private GenericPublisher HandWidget = HandTab.add("Hand Position", 0.0).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    private GenericPublisher HandLimitWidget = HandTab.add("Magnetic limit", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#009900")).withPosition(0, 1)
            .withSize(1, 1).getEntry();
    private GenericPublisher HandRotateWidget = HandTab.add("Hand Mode", -11).withPosition(0, 2).withSize(1, 1)
            .getEntry();
    private GenericPublisher HandRotatedWidget = HandTab.add("Past Hand Mode", -11).withPosition(0, 3).withSize(1, 1)
            .getEntry();

    private void updateWidgets() {
        HandWidget.setDouble(getPositioning());
        HandLimitWidget.setBoolean(getHandLimitSwitch());
        HandRotateWidget.setInteger(handPosition);

        Mode.setBoolean(Flags.ConeMode);

        CurrentR.setDouble(RPickup.getOutputCurrent());

        CurrentL.setDouble(LPickup.getOutputCurrent());
    }
}
