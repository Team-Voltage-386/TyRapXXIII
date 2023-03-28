package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PersistentShufflableDouble;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.utils.mapping.*;
import static frc.robot.Constants.SmoothingConstants.*;

public class DriverCommands extends CommandBase {

    private Drivetrain driveTrain;
    private PersistentShufflableDouble driveCurvingPower = new PersistentShufflableDouble(1, "driveCurvingPower");
    private PersistentShufflableDouble rotationCurvingPower = new PersistentShufflableDouble(3, "rotationCurvingPower");

    private double testingBoostSpeed; // comment out before tryouts
    private double driveJoystickAngle, driveMagnitude, driveJoystickMagnitude;
    public double m_driveSpeed;
    public double m_rotSpeed;
    public int m_joystickOrientationMultiplier;

    public DriverCommands(Drivetrain DT) {
        updateShufflables();
        driveTrain = DT;
        testingBoostSpeed = PSDMaxDriveSpeed.get(); // comment out before tryouts
    }

    @Override
    public void initialize() {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
        driveMagnitude = 0;
        updateShufflables();
    }

    @Override
    public void execute() {
        updateShufflables();
        updateWidget();
        if (kDriver.getRawAxis(kLeftTrigger) > kDeadband) {
            m_driveSpeed = kSlowDriveSpeed;
            m_rotSpeed = kSlowRotSpeed;
        } else {
            m_driveSpeed = kMaxDriveSpeed;
            m_rotSpeed = kMaxRotSpeed;
        }

        driveTrain.doFieldOrientation = kDriver.getRawAxis(kRightTrigger) < 0.5;        
        // driveJoystickAngle = Math.atan2(
        // orientationMultiplier*kDriver.getRawAxis(kLeftVertical),
        // kDriver.getRawAxis(kLeftHorizontal));// radians, use atan2 to avoid undefined
        // and to use range -pi to pi
        // driveJoystickMagnitude = Math.sqrt(
        // Math.pow(kDriver.getRawAxis(kLeftVertical), 2) +
        // Math.pow(kDriver.getRawAxis(kLeftHorizontal), 2));
        // driveMagnitude = mapValue(kAccelerationSmoothFactor.get(), 0, 1,
        // driveMagnitude, driveJoystickMagnitude);

        // driveTrain.xDriveTarget = Math.sin((driveJoystickAngle)) * driveMagnitude *
        // kMaxDriveSpeed.get();
        // driveTrain.yDriveTarget = (Math.cos((driveJoystickAngle))) * driveMagnitude
        // * kMaxDriveSpeed.get();
        if (driveTrain.doFieldOrientation) {
            m_joystickOrientationMultiplier = 1;
        } else {
            m_joystickOrientationMultiplier = -1;
        }
        driveTrain.xDriveTarget = mapValue(kAccelerationSmoothFactor
                .get(), 0, 1, driveTrain.xDriveTarget,
                -m_joystickOrientationMultiplier * kDriver.getRawAxis(kLeftVertical) * m_driveSpeed);
        driveTrain.yDriveTarget = mapValue(kAccelerationSmoothFactor
                .get(), 0, 1, driveTrain.yDriveTarget,
                m_joystickOrientationMultiplier * kDriver.getRawAxis(kLeftHorizontal) * m_driveSpeed);
        driveTrain.rotationTarget = -1
                * curveJoystickAxis(kDriver.getRawAxis(kRightHorizontal), rotationCurvingPower.get())
                * m_rotSpeed;

        // comment out before tryouts
        // if (kDriver.getRawAxis(kLeftTrigger) > 0.1) {
        // testingBoostSpeed += 2;
        // } else {
        // testingBoostSpeed = PSDMaxDriveSpeed.get();
        // }

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

        if (kDriver.getRawButtonPressed(kRightBumper))
            driveTrain.resetFO();
        
        //SHOULD lock robot onto a point specified in the call, robot should always be facing towards this point, no matter where the robot is.
        if(kDriver.getRawButton(kA))
            driveTrain.aimbot(6, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    // private static final ShuffleboardTab joystickInfoTab = Shuffleboard.getTab("JSINFO");
    // private static final GenericEntry leftVerticalWidget = joystickInfoTab.add("left vertical", 0).withPosition(0, 2)
    //         .withSize(1, 1)
    //         .getEntry();
    // private static final GenericEntry leftHorizontalWidget = joystickInfoTab.add("left horizontal", 0).withPosition(1, 2)
    //         .withSize(1, 1)
    //         .getEntry();
    // private static final GenericEntry rightHorizontalWidget = joystickInfoTab.add("right horizontal", 0).withPosition(2, 2)
    //         .withSize(1, 1)
    //         .getEntry();
    // private static final GenericEntry driveVectorOrientWidget = joystickInfoTab.add("driveVectorAngle", 0).withPosition(1, 1)
    //         .withSize(1, 1).getEntry();
    // private static final GenericEntry driveVectorMagnitudeWidget = joystickInfoTab.add("dvMagni", 0).withPosition(2, 1)
    //         .withSize(1, 1).getEntry();
    // private static final GenericEntry JoystickVectorMagnitudeWidget = joystickInfoTab.add("JSMagni", 0).withPosition(3, 1)
    //         .withSize(1, 1).getEntry();

    private void updateWidget() {
        // leftVerticalWidget.setDouble(kDriver.getRawAxis(kLeftVertical));
        // leftHorizontalWidget.setDouble(kDriver.getRawAxis(kLeftHorizontal));
        // rightHorizontalWidget.setDouble(kDriver.getRawAxis(kRightHorizontal));
        // driveVectorOrientWidget.setDouble(driveJoystickAngle);
        // driveVectorMagnitudeWidget.setDouble(driveMagnitude);
        // JoystickVectorMagnitudeWidget.setDouble(driveJoystickMagnitude);
    }

    private void updateShufflables() {
        if (PSDMaxDriveSpeed.detectChanges()) {
            PSDMaxDriveSpeed.subscribeAndSet();
        }
        if (PSDMaxRotSpeed.detectChanges()) {
            PSDMaxRotSpeed.subscribeAndSet();
        }
        if (kAccelerationSmoothFactor.detectChanges()) {
            kAccelerationSmoothFactor.subscribeAndSet();
        }
        if (driveCurvingPower.detectChanges()) {
            driveCurvingPower.subscribeAndSet();
        }
    }

    private double curveJoystickAxis(double input, double power) {
        return Math.signum(input) * Math.pow(Math.abs(input), power);
    }
}
