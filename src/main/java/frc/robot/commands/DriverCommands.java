package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class DriverCommands extends CommandBase {

    private Drivetrain driveTrain;

    private double testingBoostSpeed; // comment out before tryouts

    public DriverCommands(Drivetrain DT) {
        driveTrain = DT;
        testingBoostSpeed = kMaxDriveSpeed; // comment out before tryouts
    }

    @Override
    public void initialize() {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    @Override
    public void execute() {
        updateWidget();
        // HumanDriverControl=Math.abs(kDriver.getRawAxis(kLeftTrigger))<deadband;
        driveTrain.xDriveTarget = -kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
        driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
        driveTrain.rotationTarget = -Math.pow(kDriver.getRawAxis(kRightHorizontal),3) * kMaxRotSpeed;

        // comment out before tryouts
        if (kDriver.getRawAxis(kLeftTrigger) > 0.1) {
            testingBoostSpeed += 2;
        } else {
            testingBoostSpeed = kMaxDriveSpeed;
        }

        // driveTrain.xDriveTarget = -kDriver.getRawAxis(kLeftVertical) * testingBoostSpeed;
        // driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * testingBoostSpeed;

        // if (Math.abs(kDriver.getRawAxis(kRightHorizontal)) > 0.05) {
        //     driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;
        // } else {
        //     driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;
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
    private static final GenericEntry xPosWidget = mainTab.add("left horizontal", 0).withPosition(0, 2).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("left vertical", 0).withPosition(1, 2).withSize(1, 1).getEntry();
    private static final GenericEntry widget3 = mainTab.add("right horizontal", 0).withPosition(2, 2).withSize(1, 1)
            .getEntry();
    
    private void updateWidget() {
        xPosWidget.setDouble(kDriver.getRawAxis(kLeftVertical));
        yPosWidget.setDouble(kDriver.getRawAxis(kLeftHorizontal));
        widget3.setDouble(kDriver.getRawAxis(kRightHorizontal));
    }
}
