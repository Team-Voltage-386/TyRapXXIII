package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.PID;
import frc.robot.utils.apriltag;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.Field.*;
import static frc.robot.utils.Flags.*;

import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.AutoPilotConstants.*;
import static frc.robot.Constants.Limelightconstants.*;

public class DriverCommands extends CommandBase {

    private Drivetrain driveTrain;

    private Limelight limelight;
    private apriltag target = null;
    private double facingtoscore;
    private double deadband = 0.1;
    private double adjustableXDist;
    private PID autoRPID;
    private PID autoXPID;
    private PID autoYPID;

    /**teleop driver controls */
    public DriverCommands(Drivetrain DT, Limelight LL) {
        driveTrain = DT;
        limelight = LL;
        addRequirements(driveTrain);
        addRequirements(limelight);
        facingtoscore = -180;// set to 0 or 180 based on team
        adjustableXDist = 6.25;// -6.25 if on blue team
        autoRPID = new PID(kAutoRotationPID[0], kAutoRotationPID[1], kAutoRotationPID[2]);
        autoXPID = new PID(kAutoDriveXPID[0], kAutoDriveXPID[1], kAutoDriveXPID[2]);
        autoYPID = new PID(kAutoDriveYPID[0], kAutoDriveYPID[1], kAutoDriveYPID[2]);
    }

    @Override
    public void initialize() {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    @Override
    public void execute() {
        // set the flag for human control; currently the only time it is not human
        // control is when the trigger is pressed
        // when the trigger is pressed, right now it enters the primitive target lock on
        // with the apritag/retroreflective
        // humans can only drive with joystick while human driver control is on
        HumanDriverControl = Math.abs(kDriver.getRawAxis(kLeftTrigger)) < deadband;

        if (HumanDriverControl) {
            // joystick input
            driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
            driveTrain.xDriveTarget = -kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
            driveTrain.rotationTarget = -Math.pow(kDriver.getRawAxis(kRightHorizontal), 3) * kMaxRotSpeed;

            // this was the feature me and carl talked about a while ago; the robot
            // typically pivots about its center
            // BUT if we push the left bumper, the pivot point is offset
            if (kDriver.getRawButtonPressed(kLeftBumper))
                driveTrain.setOffset(-0.65, 0);
            else if (kDriver.getRawButtonReleased(kLeftBumper))
                driveTrain.setOffset(0, 0);
            // reset the odometry
            // will use apriltag if available to get proper field orientation;
            // if no apriltags: set current orientation as zero (TBD; unsure what it thinks
            // it is facing, but field oriented joystick is likely to be wrong)
            if (kDriver.getRawButtonPressed(kRightBumper)) {
                if (limelight.apriltagmode() && limelight.apriltagsAvailable())
                    driveTrain.setFO(limelightYawToDriveTrainYaw());
                else
                    driveTrain.resetFO();
            }
        }

        // align to apriltag when Left trigger is pressed and in ApriltagPipeline
        // override human control
        if (limelight.apriltagmode() && Math.abs(kDriver.getRawAxis(kLeftTrigger)) > deadband
                && limelight.apriltagsAvailable()) {
            HumanDriverControl = false;
            // target is null means we do not know which apriltag is closest yet
            // use apriltag to find current position on field, then find closest grid on the
            // field, then set target to closest grid
            if (target == null) {

                target = closestGrid(limelight.getPose()[0], limelight.getPose()[1]);// closest grid method is in
                                                                                     // constants
            }
            driveTrain.setFO(limelightYawToDriveTrainYaw());
            driveToTarget(adjustableXDist, target.y, facingtoscore, limelight.getPose()[0], limelight.getPose()[1],
                    driveTrain.getProcessedHeading());
        }
        // align to retroreflective when Left trigger is pressed and in Retroreflective
        // pipleine
        if (limelight.retroreflectivemode() && Math.abs(kDriver.getRawAxis(kLeftTrigger)) > deadband) {
            HumanDriverControl = false;
            driveToTarget(0, 0, facingtoscore, 0, 1.5 * Math.atan(Math.toRadians(limelight.tx())),
                    driveTrain.getProcessedHeading());
            // the value 1.5 times arctan should be a constant TBD
            // driving based off of target angles instead of robot position
            // x is assumed to be correct, adjust only y and keep orientation good
        }
        //switch the pipelines
        if (kDriver.getRawButtonPressed(kX)) {
            if (limelight.apriltagmode())
                limelight.setPipeline(retroreflectivepipelineindex);
            else if (limelight.retroreflectivemode())
                limelight.setPipeline(apriltagpipelineindex);
        }
    }

    /** feed limelight yaw into drivetrainyaw */ 
    public double limelightYawToDriveTrainYaw() {
        return (limelight.getPose()[5]) + 180 - 90;// limelight field coordinate system is 180, gyroscope weirdness says
                                                   // subtract 90
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    //
    /**
     * PID drive to target, feed in target position and rotation and robot position
     * and rotation
     * 
     * @param targetX
     * @param targetY
     * @param targetRot
     * @param bpx       bot pose x
     * @param bpy       bot pose y
     * @param bpr       bot pose rotation
     */
    public void driveToTarget(double targetX, double targetY, double targetRot, double bpx, double bpy, double bpr) {
        driveTrain.rotationTarget = autoRPID.calc(targetRot - bpr);
        driveTrain.xDriveTarget = autoXPID.calc(targetX - bpx);
        driveTrain.yDriveTarget = autoYPID.calc(targetY - bpy);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

}
