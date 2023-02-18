package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import frc.robot.utils.PIDShufflable;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;

public class Drive extends CommandBase {

    private final double x;
    private final double y;
    private final double h;
    private final Drivetrain dt;
    private final PIDShufflable autoPositionX = new PIDShufflable(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2],"autoPosX");
    private final PIDShufflable autoPositionY = new PIDShufflable(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2],"autoPosY");
    private final PIDShufflable autoPositionH = new PIDShufflable(kAutoHeadingPID[0], kAutoHeadingPID[1], kAutoHeadingPID[2],"autoPosH");

    public Drive(double X, double Y, double H, Drivetrain DT) {
        x = X;
        y = Y;
        h = H;
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Drive Starting");
    }

    @Override
    public void execute() {
        dt.xDriveTarget = autoPositionX.calc(x - dt.xPos);
        dt.yDriveTarget = autoPositionY.calc(y - dt.yPos);
        dt.rotationTarget = -autoPositionH.calc(dt.getHeadingError(h));

        // System.out.println("x value: " + dt.xDriveTarget + " " + "y value: " +
        // dt.yDriveTarget);
    }

    @Override
    public boolean isFinished() {
        return dt.distanceTo(x, y) < driveTolerance
                && dt.getHeadingError(h) < headingTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        dt.yDriveTarget = 0;
        dt.rotationTarget = 0;
        System.out.println("Driving Done");
    }
}
