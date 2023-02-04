package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {

    private final Drivetrain dt;
    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);

    // Might potentially need in order to slow the robot down to prevent oscillation
    // on the balance
    // private int timesSwappedCounter = 0;
    // private int timerCounter;

    boolean balanceDone = false;
    // This is zone in degrees when the charging station is considered balanced
    private double balanceTarget = 2.5;

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Balancing Starting");
    }

    @Override
    public void execute() {
        pigeon.getYawPitchRoll(ypr);
        // The Pigeon is mounted 90 degrees off, so pitch and roll are reversed
        if (Math.abs(ypr[2]) > balanceTarget) {
            // timerCounter = 0;
            if (ypr[2] > balanceTarget) {
                dt.xDriveTarget = 0.5;
                // Might need in order to slow down balancing
                // dt.xDriveTarget = 0.25 * Math.pow(0.5, (timesSwappedCounter));
            }
            if (ypr[2] < balanceTarget) {
                dt.xDriveTarget = -0.5;
                // Might need in order to slow down balancing
                // dt.xDriveTarget = -0.25 * Math.pow(0.5, (timesSwappedCounter));
            }
        }
        // timesSwappedCounter++;
    }

    @Override
    public boolean isFinished() {
        // Checks to make sure that the robot pitch is within the acceptable range
        return Math.abs(ypr[2]) < balanceTarget;
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
        System.out.println("Balancing Done");
    }
}
