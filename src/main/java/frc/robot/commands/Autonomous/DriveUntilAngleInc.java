package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import frc.robot.utils.mapping;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;

/**
 * Drives until the robot reaches a target angle (A). If the robot doesnt reach the angle within the specified distance (sqrt(X^2 + Y^2), then the command stops.)
 */
public class DriveUntilAngleInc extends CommandBase {

    private final double x;
    private final double y;
    private final double h;
    private final double speed;
    private final Drivetrain dt;
    private final double targAngle;
    private final int yprAxis;
    private final PID autoPositionX = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private final PID autoPositionY = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private final PID autoPositionH = new PID(kAutoHeadingPID[0], kAutoHeadingPID[1], kAutoHeadingPID[2]);
    private boolean flag = false;

    public DriveUntilAngleInc(double X, double Y, double H, double S, Drivetrain DT, double A, int ypra) {
        x = X;
        y = Y;
        h = H;
        speed = S;
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
        SmartDashboard.putNumber("angle", dt.ypr[yprAxis]);
        SmartDashboard.putBoolean("error reached", dt.distanceTo(x, y) < driveTolerance && dt.getHeadingError(h) < headingTolerance);
        
        dt.xDriveTarget = mapping.clamp(autoPositionX.calc(x - dt.xPos),-kMaxDriveSpeed*speed,kMaxDriveSpeed*speed);
        dt.yDriveTarget = mapping.clamp(autoPositionY.calc(y - dt.yPos), -kMaxDriveSpeed*speed, kMaxDriveSpeed*speed);
        dt.rotationTarget = mapping.clamp(-autoPositionH.calc(dt.getHeadingError(h)), -kMaxRotSpeed,kMaxRotSpeed);


    }

    @Override
    public boolean isFinished() {
        flag = dt.distanceTo(x, y) < driveTolerance && dt.getHeadingError(h) < headingTolerance;
        if(Math.abs(dt.ypr[yprAxis]) > targAngle || flag) {
            return true;
        }
        else 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        dt.yDriveTarget = 0;
        dt.rotationTarget = 0;
        System.out.println("Driving Done");
    }
}
