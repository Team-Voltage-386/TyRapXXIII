package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
    // Will need to update with correct constants/CAN IDs
    private CANSparkMax gripperMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax gripperMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    private PneumaticsControlModule pcm = new PneumaticsControlModule();
    private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 3);
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private double releaseSpeed = 0.1; // wheel speed when releasing a piece
    private double closeSpeed = -0.1; // wheel speed when capturing a piece

    public GripperSubsystem() {
        doubleSolenoid.set(Value.kOff); // might later want the gripper to default to open/closed?
        gripperMotor1.setInverted(true); // will need to figure out which motor is inverted
    }

    public void releaseGripper() {
        // open pneumatics
        doubleSolenoid.set(Value.kForward);
        // move wheels to drop object
        gripperMotor1.set(releaseSpeed);
        gripperMotor2.set(releaseSpeed);
    }

    public void closeGripper() {
        // close pneumatics
        doubleSolenoid.set(Value.kReverse);
        // move wheels to suck in game piece
        gripperMotor1.set(closeSpeed);
        gripperMotor2.set(closeSpeed);
    }

    public void reorientGripper() {
        // move gripper to be parrallel with ground. Use gyro?
    }

    public void moveGripperSideways() {
        // move gripper to be perpendicular with ground. Use gyro?
    }

    @Override
    public void periodic() {

    }

}
