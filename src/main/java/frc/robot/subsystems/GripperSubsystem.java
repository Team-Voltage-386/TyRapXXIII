package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

//Stuff to work on based on mechanical discussion: the gripper should always be closed when in cone mode and 
//always be open in cube mode. There should be enough power while picking up the cone to force open the gripper. 
//The encoder used for the gripper rotation will be a hall-effect sensor read off of the PLG motor. Mechanical 
//mentioned that belt slipping may be a problem, but TBD. When the robot is is possession of a cone, the current 
//should be consistently higher than normal current. There will probably be a sweet spot to determine whether
//the robot is in possession of a game piece. Values for holding speed for cubes and cones and picking up speed 
//will need to be determined experimentally

public class GripperSubsystem extends SubsystemBase {
    // Will need to update with correct constants/CAN IDs
    private CANSparkMax gripperMotor1 = new CANSparkMax(kRightPickupID, MotorType.kBrushless);
    private CANSparkMax gripperMotor2 = new CANSparkMax(kLeftPickupID, MotorType.kBrushless);
    private PneumaticsControlModule pcm = new PneumaticsControlModule();
    private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kSolenoidForward, kSolenoidReverse);
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private double releaseSpeed = 0.05; // wheel speed when releasing a piece
    private double closeSpeed = -0.05; // wheel speed when holding a piece
    private double intakeSpeed = 0.1; // wheel speed when capturing a piece

    private double holdingCurrent = 10; // Current spike that determines whether the robot is in possession of a game
                                        // piece

    public GripperSubsystem() {
        doubleSolenoid.set(Value.kOff); // might later want the gripper to default to open/closed?
        gripperMotor1.setInverted(true); // will need to figure out which motor is inverted, if any are inverted at all
    }

    public void releaseGripper() {
        // might not need to open the pneumatics to release
        doubleSolenoid.set(Value.kForward);
        // move wheels to drop object
        gripperMotor1.set(releaseSpeed);
        gripperMotor2.set(releaseSpeed);
    }

    public void holdGripper() {
        doubleSolenoid.set(Value.kReverse);
        // move wheels to suck in game piece
        gripperMotor1.set(closeSpeed);
        gripperMotor2.set(closeSpeed);
    }

    public void intakeGripper() {
        // open gripper
        doubleSolenoid.set(Value.kForward);
        //
        gripperMotor1.set(intakeSpeed);
        gripperMotor1.set(intakeSpeed);

        // Might want this logic in the command instead of the subsystem
        if (isGamePieceDetected()) {
            closeGripper();
        }
    }

    public void reorientGripper() {
        // move gripper to be parrallel with ground. Use gyro?
    }

    public void moveGripperSideways() {
        // move gripper to be perpendicular with ground. Use gyro?
    }

    public boolean isGamePieceDetected() {
        if (gripperMotor1.getOutputCurrent() >= holdingCurrent || gripperMotor2.getOutputCurrent() >= holdingCurrent) {
            return true;
        } else {
            return false;
        }
    }

    //This will probably be used to close the gripper when entering cone mode
    public void closeGripper() {
        doubleSolenoid.set(Value.kForward);
    }

    public void openGripper() {
        doubleSolenoid.set(Value.kReverse);
    }
    @Override
    public void periodic() {
        updateWidgets();
    }

    public void updateWidgets() {
        isGamePieceDetectedWidget.setBoolean(isGamePieceDetected());
    }

    private static final ShuffleboardTab gripperTab = Shuffleboard.getTab("Gripper Tab");
    private static final GenericEntry isGamePieceDetectedWidget = gripperTab.add("Is Game Piece Detected", false)
            .getEntry();

}
