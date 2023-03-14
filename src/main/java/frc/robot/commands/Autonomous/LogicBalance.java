package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class LogicBalance extends CommandBase {

    private final Drivetrain dt;
    // Store gyro values from pigeon
    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);

    boolean balanceDone = false;
    // This is zone in degrees when the charging station is considered balanced
    private double balanceTarget = 5;
    // Counts the number of times that the direction has changed (going forward and
    // backward on the charge station)
    private int numTimesDirectionChanged = 0;
    // This boolean is used to determine when the robot changes direction
    private boolean wasGoingForward = true;
    // This value is used to slow down the drive as the balance progresses
    public double driveMultiplier;

    // variables to display on shuffleboardQ
    private boolean isDrivingForward;

    public LogicBalance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        // System.out.println("Balancing Starting");
    }

    @Override
    public void execute() {
        // The Pigeon is mounted 90 degrees off, so pitch and roll are reversed
        pigeon.getYawPitchRoll(ypr);
        // Slows down the robot as the balance progresses
        driveMultiplier = Math.pow(0.6, numTimesDirectionChanged);

        if (Math.abs(ypr[2]) > balanceTarget) {
            if (ypr[2] > balanceTarget) {
                // driveMultiplier = Math.pow(0.8, numTimesDirectionChanged);
                dt.xDriveTarget = -0.5 * driveMultiplier;
                // Increments numTimesDirectionChanged
                if (!wasGoingForward) {
                    wasGoingForward = true;
                    numTimesDirectionChanged++;
                }
                isDrivingForward = true;
            } else if (ypr[2] < balanceTarget) {
                // driveMultiplier = Math.pow(0.8, numTimesDirectionChanged);
                dt.xDriveTarget = 0.5 * driveMultiplier;
                // Increments numTimesDirectionChanged
                if (wasGoingForward) {
                    wasGoingForward = false;
                    numTimesDirectionChanged++;
                }
                isDrivingForward = false;
            }
        } else {
            // circle - lock the wheels when the charging station is balanced
            dt.xDriveTarget = 0;
        }
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
