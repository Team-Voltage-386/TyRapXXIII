package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;

/**
 * Drives until the robot reaches a target angle (A). If the robot doesnt reach the angle within the specified distance (sqrt(X^2 + Y^2), then the command stops.)
 */
public class DriveUntilAngle extends CommandBase {

    private final double x;
    private final double y;
    private final double h;
    private final Drivetrain dt;
    private final double targAngle;
    private final int yprAxis;
    private boolean stop = false;
    private final PID autoPositionX = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private final PID autoPositionY = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private final PID autoPositionH = new PID(kAutoHeadingPID[0], kAutoHeadingPID[1], kAutoHeadingPID[2]);

    public DriveUntilAngle(double X, double Y, double H, Drivetrain DT, double A, int ypra) {
        x = X;
        y = Y;
        h = H;
        dt = DT;
        targAngle = A;
        yprAxis = ypra;
    }

    @Override
    public void initialize() {
        System.out.println("Drive Starting");
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("distance", dt.distanceTo(x, y));
        SmartDashboard.putNumber("pitch", dt.ypr[2]);
        SmartDashboard.putBoolean("stop", stop);
        SmartDashboard.putBoolean("error reached", dt.distanceTo(x, y) < driveTolerance && dt.getHeadingError(h) < headingTolerance);
        if(stop == true) {
            dt.xDriveTarget = 0;
            dt.yDriveTarget = 0;
            dt.rotationTarget = 0;
            System.out.println("Error: max distance reached, did not reach angle.");
        }
        else {
            dt.xDriveTarget = autoPositionX.calc(x - dt.xPos);
            dt.yDriveTarget = autoPositionY.calc(y - dt.yPos);
            dt.rotationTarget = -autoPositionH.calc(dt.getHeadingError(h));
        }
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("distance", dt.distanceTo(x, y));
        SmartDashboard.putNumber("pitch", dt.ypr[2]);
        SmartDashboard.putBoolean("stop", stop);
        SmartDashboard.putBoolean("error reached", dt.distanceTo(x, y) < driveTolerance && dt.getHeadingError(h) < headingTolerance);

        if(Math.abs(targAngle - Math.abs(dt.ypr[yprAxis])) < 0.2) {
            return true;
        }
        else {
            if(dt.distanceTo(x, y) < driveTolerance && dt.getHeadingError(h) < headingTolerance){
                stop = true;
            }
            else stop = false;

            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        dt.yDriveTarget = 0;
        dt.rotationTarget = 0;
        System.out.println("Driving Done");
    }
}
