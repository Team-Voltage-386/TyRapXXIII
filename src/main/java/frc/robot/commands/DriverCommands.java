package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.utils.mapping.*;
import static frc.robot.Constants.SmoothingConstants.*;

public class DriverCommands extends CommandBase {

    private Drivetrain driveTrain;

    private double testingBoostSpeed; // comment out before tryouts

    public DriverCommands(Drivetrain DT) {
        updateShufflables();
        driveTrain = DT;
        testingBoostSpeed = kMaxDriveSpeed.get(); // comment out before tryouts
    }

    @Override
    public void initialize() {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
        updateShufflables();
    }

    @Override
    public void execute() {
        updateShufflables();
        updateWidget();
        // HumanDriverControl=Math.abs(kDriver.getRawAxis(kLeftTrigger))<deadband;
        driveTrain.xDriveTarget = mapValue(kAccelerationSmoothFactor.get(), 0, 1, driveTrain.xDriveTarget,
                -kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed.get());
        driveTrain.yDriveTarget = mapValue(kAccelerationSmoothFactor.get(), 0, 1, driveTrain.yDriveTarget,
                kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed.get());
        driveTrain.rotationTarget = mapValue(1, 0, 1, driveTrain.yDriveTarget,
                -Math.pow(kDriver.getRawAxis(kRightHorizontal), 3) * kMaxRotSpeed.get());

        // comment out before tryouts
        if (kDriver.getRawAxis(kLeftTrigger) > 0.1) {
            testingBoostSpeed += 2;
        } else {
            testingBoostSpeed = kMaxDriveSpeed.get();
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

        if (kDriver.getRawButtonPressed(kRightBumper))
            driveTrain.resetFO();
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

    private static final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private static final GenericEntry xPosWidget = mainTab.add("left horizontal", 0).withPosition(0, 2).withSize(1, 1)
            .getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("left vertical", 0).withPosition(1, 2).withSize(1, 1)
            .getEntry();
    private static final GenericEntry widget3 = mainTab.add("right horizontal", 0).withPosition(2, 2).withSize(1, 1)
            .getEntry();

    private void updateWidget() {
        xPosWidget.setDouble(kDriver.getRawAxis(kLeftVertical));
        yPosWidget.setDouble(kDriver.getRawAxis(kLeftHorizontal));
        widget3.setDouble(kDriver.getRawAxis(kRightHorizontal));
    }

    private void updateShufflables(){
        if(kMaxDriveSpeed.detectChanges()){
            kMaxDriveSpeed.subscribeAndSet();
        }
        if(kMaxRotSpeed.detectChanges()){
            kMaxRotSpeed.subscribeAndSet();
        }
        if(kAccelerationSmoothFactor.detectChanges()){
            kAccelerationSmoothFactor.subscribeAndSet();
        }
    }

}
