package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Enumeration;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PersistentShufflableDouble;

import static frc.robot.Constants.HandConstants.*;
import static frc.robot.utils.Flags.*;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import static frc.robot.Constants.*;
import static frc.robot.utils.mapping.*;

import static frc.robot.utils.Flags.*;

public class Hand extends SubsystemBase {
    /*
     * handPosition is which of the three possible possitions the hand is in
     * if handPosition = -1 it is rotated counter-clockwise
     * if handPosition = 0 it is centered and can be retracted
     * if handPosition = 1 it it rotated clockwise
     */

    public int handPosition;
    public handIntakeStates intakeCurrentTask = handIntakeStates.doNothing;
    double intakeSpeed;
    double ejectSpeed;
    double stowSpeed;

    boolean handTurningClockwise;
    double targHandPos = 0;
    public double wristTarget;
    public PIDController wristPIDController;

    // Declaration of motors and pnumatics
    public DoubleSolenoid pcmCompressor;
    public TalonSRX HandRotationalMotor;

    // Limit declaration
    private DigitalInput HandLimitSwitch; // LIMIT READING TRUE MEANS SWTICH NOT HIT
    public boolean HandHitLimit; // HandHitLimit WILL READ TRUE WHEN IT HITS THE LIMIT

    //PID values
    private double maxRPM = 5700;

    private double kP; 
    private double kI;
    private double kD; 
    private double kIz; 
    private double kFF; 
    private double kMaxOutput; 
    private double kMinOutput;

    private double setPoint;

    private ShuffleboardTab pidTab = Shuffleboard.getTab("Test PID");
    private SimpleWidget PValue, IValue, DValue, IZone, FeedForward, MaxOutput, MinOutput, SetPoint, VelocityValue;

    private SparkMaxPIDController SparkPIDcontrols;
    private RelativeEncoder m1_encoder;
    

    public Hand() {
        pcmCompressor = new DoubleSolenoid(kDoubleSolenoidModule, PneumaticsModuleType.CTREPCM, kSolenoidForward,
                kSolenoidReverse);
        HandRotationalMotor = new TalonSRX(kHandRotator);
        HandLimitSwitch = new DigitalInput(kHandLimitSwitch);
        HandRotationalMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        handPosition = 0;

        RPickup = new CANSparkMax(HandConstants.kRightPickupID, MotorType.kBrushless);
        LPickup = new CANSparkMax(HandConstants.kLeftPickupID, MotorType.kBrushless);
        //wristPIDController = new PIDController(kWristPID[0], kWristPID[1], kWristPID[2]);

        //OLD PID Stuff
        m1_encoder = RPickup.getEncoder();
        //NEW PID Stuff
        SparkPIDcontrols = RPickup.getPIDController();

        kP = 1e-10; 
        kI = 0.1;
        kD = 0.0001; 
        kIz = 1; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        PValue = pidTab.add("P Value", kP);
        IValue = pidTab.add("I Value", kI);
        DValue = pidTab.add("D Value", kD);
        IZone= pidTab.add("I Zone", kIz);
        FeedForward = pidTab.add("Feed Forward", kFF);
        MaxOutput = pidTab.add("Max Output", kMaxOutput);
        MinOutput = pidTab.add("Min Output", kMinOutput);
        SetPoint = pidTab.add("Set Point", setPoint);
        VelocityValue = pidTab.add("Velocity", m1_encoder.getVelocity());

        SparkPIDcontrols.setP(kP);
        SparkPIDcontrols.setI(kI);
        SparkPIDcontrols.setD(kD);
        SparkPIDcontrols.setIZone(kIz);
        SparkPIDcontrols.setFF(kFF);
        SparkPIDcontrols.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void periodic() {
        //setHandMotor();
        //runIntakeMotorState();
        updateWidgets();
        updatePID();
    }

    public double getWristAngle() {
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
            RPickup.setSmartCurrentLimit(35, 15);
            LPickup.setSmartCurrentLimit(35, 15);
            pcmCompressor.set(Value.kForward);

        } else {
            RPickup.setSmartCurrentLimit(5, 10);
            LPickup.setSmartCurrentLimit(5, 10);
            pcmCompressor.set(Value.kReverse);
        }
    }

    public static enum handIntakeStates {
        letitgo, intake, doNothing, stow
    }

    public void IntakeMotorControl(handIntakeStates intakeTask) {
        intakeCurrentTask = intakeTask;
        if (ConeMode) {
            intakeSpeed = kConeIntakeSpeed;
            ejectSpeed = -kConeIntakeSpeed;
            stowSpeed = kCubeStowSpeed;
        } else {
            intakeSpeed = kCubeIntakeSpeed;
            ejectSpeed = -kCubeIntakeSpeed - .2;
            stowSpeed = kCubeStowSpeed;
        }

    }

    private void runIntakeMotorState() {
        switch (intakeCurrentTask) {
            case intake:
                RPickup.set(intakeSpeed);
                LPickup.set(intakeSpeed);
                break;
            case letitgo:
                RPickup.set(ejectSpeed);
                LPickup.set(ejectSpeed);
                break;
            case stow:
                RPickup.set(stowSpeed);
                LPickup.set(stowSpeed);
                break;
            default:
                RPickup.set(0);
                LPickup.set(0);
                break;

        }
    }

    public boolean canRetract() {
        return (Math.abs(getWristAngle()) < 5.0);
    }

    private void setHandMotor() {
        // double maxPower = kMaxWristPower;
        // double minPower = -kMaxWristPower;
        // wristTarget = 0;
        // if (handCanRotate && ConeMode) {
        // if (handPosition == -1)
        // wristTarget = -kWristSideTargetAngle;
        // else if (handPosition == 1)
        // wristTarget = kWristSideTargetAngle;
        // }
        // if (getHandLimitSwitch()) {
        // //upper limit
        // if (getWristAngle() > 0) {
        // maxPower = 0;
        // } else {
        // minPower = 0;
        // }
        // }
        // HandRotationalMotor.set(ControlMode.PercentOutput,
        // clamp(wristPIDController.calculate(getWristAngle(),
        // wristTarget), minPower, maxPower));

        // old
        if (handCanRotate) {
            if (handPosition == 1 && !getHandLimitSwitch() && getWristAngle() < 75) {
                HandRotationalMotor.set(ControlMode.PercentOutput, -kRotationSpeed);
            } else if (handPosition == 0 && getWristAngle() > 0 && !canRetract() &&
                    !getHandLimitSwitch()) {
                HandRotationalMotor.set(ControlMode.PercentOutput, kRotationSpeed);
            } else if (handPosition == 0 && getWristAngle() < 0 && !canRetract() &&
                    !getHandLimitSwitch()) {
                HandRotationalMotor.set(ControlMode.PercentOutput, -kRotationSpeed);
            } else if (handPosition == -1 && !getHandLimitSwitch() && getWristAngle() > -75) {
                HandRotationalMotor.set(ControlMode.PercentOutput, kRotationSpeed);
                // fix hand positions
            } else if (handPosition == 1 && getWristAngle() > 78) {
                HandRotationalMotor.set(ControlMode.PercentOutput, kRotationSpeed);
            } else if (handPosition == -1 && getWristAngle() < -78) {
                HandRotationalMotor.set(ControlMode.PercentOutput, -kRotationSpeed);
                // do nothing
            } else {
                HandRotationalMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            HandRotationalMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setRotateHand(boolean isRightBumper) {
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

    /** Returns true when the limit switch hits its limit */
    public boolean getHandLimitSwitch() {
        return !HandLimitSwitch.get();
    }

    public void setLimitClear() {

    }

    private GenericPublisher HandAngleWidget = HandTab.add("Hand Angle", 0.0).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    private GenericPublisher HandLimitWidget = HandTab.add("Magnetic limit", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#009900")).withPosition(0, 1)

            .withSize(1, 1).getEntry();
    private GenericPublisher HandStateWidget = HandTab.add("Hand Mode", -11).withPosition(0, 2).withSize(1, 1)
            .getEntry();
    private GenericPublisher handIntakeTask = HandTab.add("Intake Task", "").withPosition(1, 2).getEntry();

    public String whatIsIntakeDoing() {
        switch (intakeCurrentTask) {
            case intake:
                return "intake";
            case letitgo:
                return "lettingGo";
            case doNothing:
                return "doNothing";
            case stow:
                return "stow";
            default:
                return "uh oh";
        }
    }

    private void updateWidgets() {
        HandAngleWidget.setDouble(getWristAngle());
        HandLimitWidget.setBoolean(getHandLimitSwitch());
        HandStateWidget.setInteger(handPosition);
        Mode.setBoolean(ConeMode);
        CurrentR.setDouble(RPickup.getOutputCurrent());
        CurrentL.setDouble(LPickup.getOutputCurrent());
        handIntakeTask.setString(whatIsIntakeDoing());
    }

    private void initializePID() {
        for (PersistentShufflableDouble i : WristPIDPSDs) {
            if (i.detectChanges())
                i.subscribeAndSet();
        }
    }

    private void writePID() {
        wristPIDController.setP(WristPIDPSDs[0].get());
        wristPIDController.setI(WristPIDPSDs[1].get());
        wristPIDController.setD(WristPIDPSDs[2].get());
    }

    public void TestPID(double RotateAngle)
    {
        SparkPIDcontrols.setReference(RotateAngle, CANSparkMax.ControlType.kSmartVelocity);
    }

    private void updatePID()
    {
        //System.out.println("PID UPDATING"+m1_encoder.getVelocity());
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", m1_encoder.getVelocity());
        double p = PValue.getEntry().getDouble(0);
        double i = IValue.getEntry().getDouble(0);
        double d = DValue.getEntry().getDouble(0);
        double iz = IZone.getEntry().getDouble(0);
        double ff = FeedForward.getEntry().getDouble(0);
        double max = MaxOutput.getEntry().getDouble(0);
        double min = MinOutput.getEntry().getDouble(0);

        System.out.println(SparkPIDcontrols.getP());

        if (kP != p)
        {
            kP = p;
            SparkPIDcontrols.setP(kP);
        }
        if (kI != i)
        {
            kI = i;
            SparkPIDcontrols.setI(kI);
        }
        if (kD != d)
        {
            kD = d;
            SparkPIDcontrols.setD(kD);
        }
        if (kIz != iz)
        {
            kIz = iz;
            SparkPIDcontrols.setIZone(kIz);
        }
        if (kFF != ff)
        {
            kFF = ff;
            SparkPIDcontrols.setFF(kFF);
        }
        if (kMaxOutput != max || kMinOutput != min)
        {
            kMaxOutput = max;
            kMinOutput = min;
            SparkPIDcontrols.setOutputRange(kMinOutput, kMaxOutput);
        }    
    }
}
