package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPI_Drivetrain extends SubsystemBase{

    public double ypr[] = new double[3];

    public Pigeon2 gyro = new Pigeon2(kIMUid);

    public WPI_SwerveModule[] modules = { RightFrontWPI, RightRearWPI, LeftRearWPI, LeftFrontWPI};


    public WPI_Drivetrain() {
        //waits for gyro to calibrate before zeroing it.
        //does this on a different thread as to no interupt current code.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //*zeros heading */
    public void zeroHeading() {
        gyro.setYaw(0);
    }
    
    public double getHeading() {
        return Math.IEEEremainder((gyro.getYaw()), 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        gyro.getYawPitchRoll(ypr);
    }

    public void stopModules() {
        LeftFrontWPI.stop();
        LeftRearWPI.stop();
        RightFrontWPI.stop();
        RightRearWPI.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxDriveSpeed);
        LeftFrontWPI.setDesiredState(desiredStates[0]);
        LeftRearWPI.setDesiredState(desiredStates[1]);
        RightFrontWPI.setDesiredState(desiredStates[2]);
        RightRearWPI.setDesiredState(desiredStates[3]);
    }
}
