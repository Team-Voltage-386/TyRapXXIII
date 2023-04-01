package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WPI_Drivetrain extends SubsystemBase{

    public double ypr[] = new double[3];

    public Pigeon2 gyro = new Pigeon2(kIMUid);

    public WPI_SwerveModule[] modules = {RightFrontWPI, RightRearWPI, LeftRearWPI, LeftFrontWPI};

    public SwerveModulePosition[] modulePositions;

    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), modulePositions);


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
    
    //*get heading of robot in degrees*/
    public double getHeading() {
        return Math.IEEEremainder((gyro.getYaw()), 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getHeadingRotation2d(), modulePositions, pose);
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                this.kinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }

    @Override
    public void periodic() {
        //update odo and swerve module data/positions
        for(int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
        }
        odometer.update(getHeadingRotation2d(), modulePositions);

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
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
