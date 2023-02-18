package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;

public class LogicBalance extends CommandBase{
    private final Drivetrain dt;

    //Note: pigeon is off by 90 degrees, so account for this when calling ypr
    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);

    //For the balance to be done, the robot has to be within the balanceTarget zone for 0.2 seconds (20 executions)
    boolean balanceDone = false;
    int timerCounter = 0;

    //This is the zone in degrees when the charging station is considered balanced
    private double balanceTarget = 2.5;

    //These varaibles together are used in order to slow the robot down as it balances in order to minimize oscillations
    private int numTimesDirectionChanged = 0; //Keeps track of how many times the robot has changed directions (goign forward to going backward)
    private double slowingValue = 0.8; //This value is raised to the power of the numTimesDirectionChanged
    private double balancingSpeed = 1.5; //This is the base speed before slowing down when balancing

    //Makes sure that the robot is correcting the balance in the right direction
    private boolean wasGoingForward = true;

    //The value by which the drive speed is slowed down
    private double driveMultiplier;

    public LogicBalance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        balanceTarget = 2.5;
    }

    public void execute() {
        pigeon.getYawPitchRoll(ypr);
        driveMultiplier = Math.pow(slowingValue, numTimesDirectionChanged);

        if (Math.abs(ypr[2]) > balanceTarget) {
            if (ypr[2] > balanceTarget) { // The robot is tilted up, and should be driving forward
                dt.xDriveTarget = balancingSpeed * driveMultiplier;
                //Keeps track of the direction that the balance needs to be corrected
                if (!wasGoingForward) {
                    wasGoingForward = true;
                    numTimesDirectionChanged++;
                }
            } else if (ypr[2] < balanceTarget) { //The robot is tilted down, and should be driving backwards
                dt.xDriveTarget = -balancingSpeed * driveMultiplier;
                //Keeps track of the direction that the balance needs to be corrected
                if (wasGoingForward) {
                    wasGoingForward = false;
                    numTimesDirectionChanged++;
                }
            }
        } else {
            //The timer utilizes the fact that the execute is run at a known frequency to calculate the time elapsed;
            timerCounter++;
            dt.xDriveTarget = 0;
            if (timerCounter >= 20) {
                balanceDone = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return balanceDone;
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
    }
}
