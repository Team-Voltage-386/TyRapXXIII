package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
// import frc.robot.utils.PIDShufflable;
import frc.robot.utils.mapping;
// import frc.robot.utils.mapping.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;

public class Drive extends CommandBase {

    private final double x;
    private final double y;
    private final double h;
    private final Drivetrain dt;
    
    /**
     * achieve relative x, y, and H (target x, y and heading)
     * @param X
     * @param Y
     * @param H
     * @param DT
     */
    public Drive(double X, double Y, double H, Drivetrain DT) {
        x = X;
        y = Y;
        h = H;
        dt = DT;
    }

    @Override
    public void initialize() {
        // autoPositionX.shuffleUpdatePID();
        // autoPositionY.shuffleUpdatePID();
        // autoPositionH.shuffleUpdatePID();
        System.out.println("Drive Starting");
    }

    @Override
    public void execute() {
        dt.xDriveTarget = mapping.clamp(autoPositionX.calc(x - dt.xPos),-kMaxDriveSpeed,kMaxDriveSpeed);
        dt.yDriveTarget = mapping.clamp(autoPositionY.calc(y - dt.yPos), -kMaxDriveSpeed, kMaxDriveSpeed);
        dt.rotationTarget = mapping.clamp(autoPositionH.calc(dt.getHeadingError(h)), -kMaxRotSpeed,kMaxRotSpeed);

        // System.out.println("x value: " + dt.xDriveTarget + " " + "y value: " + dt.yDriveTarget);
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
