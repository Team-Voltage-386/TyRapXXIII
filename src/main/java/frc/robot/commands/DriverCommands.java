package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
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

    private double facingtoscore;
    private double deadband = 0.1;
    private double adjustableXDist;
    private PID autoRPID;
    private PID autoXPID;
    private PID autoYPID;

    public DriverCommands(Drivetrain DT) {
        driveTrain = DT;
        addRequirements(driveTrain);
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
        HumanDriverControl = Math.abs(kDriver.getRawAxis(kLeftTrigger)) < deadband;
        driveTrain.xDriveTarget = -kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
        driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
        driveTrain.rotationTarget = -Math.pow(kDriver.getRawAxis(kRightHorizontal), 3) * kMaxRotSpeed;
        driveTrain.rotationTarget = -Math.pow(kDriver.getRawAxis(kRightHorizontal), 3) * kMaxRotSpeed;

        if (HumanDriverControl) {
            driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
            driveTrain.xDriveTarget = kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
            driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;
        // comment out before tryouts
        if (kDriver.getRawAxis(kLeftTrigger) > 0.1) {
            testingBoostSpeed += 2;
        } else {
            testingBoostSpeed = kMaxDriveSpeed;
        }

        // driveTrain.xDriveTarget = -kDriver.getRawAxis(kLeftVertical) *
        // testingBoostSpeed;
        // driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) *
        // testingBoostSpeed;

        // if (Math.abs(kDriver.getRawAxis(kRightHorizontal)) > 0.05) {
        // driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) *
        // kMaxRotSpeed;
        // } else {
        // driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) *
        // kMaxRotSpeed;
        // }

            if (kDriver.getRawButtonPressed(kLeftBumper))
                driveTrain.setOffset(-0.65, 0);
            else if (kDriver.getRawButtonReleased(kLeftBumper))
                driveTrain.setOffset(0, 0);

            if (kDriver.getRawButtonPressed(kRightBumper)) {
                driveTrain.resetFO();
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // drive to target, feed in target position and rotation and robot position and
    // rotation
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
