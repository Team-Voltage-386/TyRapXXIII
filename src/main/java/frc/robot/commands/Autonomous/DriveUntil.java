package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.AutoConstants.*;

public class DriveUntil extends CommandBase {

    private final PID autoPositionY = new PID(kAutoPositionPID[0], kAutoPositionPID[1], kAutoPositionPID[2]);
    private double ypr[] = new double[3];
    // Might instead need to use WPI_PigeonIMU in order to get access to a reset
    // method
    public Pigeon2 pigeon = new Pigeon2(kIMUid);
    private final Drivetrain dt;

    // This variable is used to store when the robot detects that it has started
    // driving up the charging pad
    private boolean changeDetected;

    private int timerCounter = 0;

    private boolean isForward;

    private Timer finishTimer = new Timer();

    public DriveUntil(boolean ISFORWARD, Drivetrain DT) {
        dt = DT;
        isForward = ISFORWARD;
    }

    @Override
    public void initialize() {
        finishTimer.stop();
        finishTimer.reset();
        System.out.println("Driving until done starting");
    }

    @Override
    public void execute() {

        pigeon.getYawPitchRoll(ypr);
        // Pigeon is mounted 90 degrees off, so use roll instead of pitch

        if (Math.abs(ypr[2]) > 12 && Math.abs(ypr[2]) <= 30) {
            timerCounter++;
            if (timerCounter > 13) {
                changeDetected = true;
            }
        } else {
            // Changing this will change the speed of the robot's approach
            if (isForward) {
                dt.xDriveTarget = 0.7;
            } else {
                dt.xDriveTarget = -0.7;
            }

        }
        // PID to keep the robot driving straight
        // dt.yDriveTarget = autoPositionY.calc(0 - dt.yPos);
        checkFinished();
    }

    private void checkFinished() {
        if (changeDetected)
            finishTimer.start();
    }

    @Override
    public boolean isFinished() {
        return finishTimer.hasElapsed(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        dt.xDriveTarget = 0;
        System.out.println("Driving until done complete");
    }
}
