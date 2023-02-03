package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.AutoConstants.*;

public class DriveUntil extends CommandBase {
    private final Drivetrain dt;
    private boolean changeDetected;
    private final PID autoPositionY = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);

    public DriveUntil(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pigeon.getYawPitchRoll(ypr);
        if (Math.abs(ypr[2]) >= 11) {
            changeDetected = true;
        } else {
            dt.xDriveTarget = 2;
        }
        dt.yDriveTarget = autoPositionY.calc(0 - dt.yPos);
    }

    @Override
    public boolean isFinished() {
        return changeDetected;
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        System.out.println("Driving until done complete");
    }
}
