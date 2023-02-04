package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private int numTimesDirectionChanged = 0;
    private boolean wasGoingForward = true;
    public double driveMultiplier;

    // variables to display on shuffleboard
    private boolean isDrivingForward;

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Balancing Starting");
    }

    @Override
    public void execute() {
        // The Pigeon is mounted 90 degrees off, so pitch and roll are reversed
        pigeon.getYawPitchRoll(ypr);
        // Slows down the robot as the balance progresses
        driveMultiplier = Math.pow(0.5, numTimesDirectionChanged);

        if (Math.abs(ypr[2]) > balanceTarget) {
            if (ypr[2] > balanceTarget) {
                dt.xDriveTarget = 0.5 * driveMultiplier;
                // Increments numTimesDirectionChanged
                if (!wasGoingForward) {
                    wasGoingForward = true;
                    numTimesDirectionChanged++;
                }
                isDrivingForward = true;
            } else if (ypr[2] < balanceTarget) {
                dt.xDriveTarget = -0.5 * driveMultiplier;
                // Increments numTimesDirectionChanged
                if (wasGoingForward) {
                    wasGoingForward = false;
                    numTimesDirectionChanged++;
                }
                isDrivingForward = false;
            }
        } else {
            dt.xDriveTarget = 0;
        }
        // timesSwappedCounter++;
        SmartDashboard.putBoolean("Is Driving Forward", isDrivingForward);
        SmartDashboard.putNumber("Number of Times Direction Changed", numTimesDirectionChanged);
        SmartDashboard.putNumber("driveMultiplier", driveMultiplier);
        SmartDashboard.putNumber("Balance Target", balanceTarget);
    }

    @Override
    public boolean isFinished() {
        // Checks to make sure that the robot pitch is within the acceptable range
        // return Math.abs(ypr[2]) < balanceTarget;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
        System.out.println("Balancing Done");
    }
}
