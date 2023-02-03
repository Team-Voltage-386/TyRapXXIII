package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
    private double balanceTarget = 2.5;
    private final Drivetrain dt;
    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);
    private int timesSwappedCounter = 0;
    private int timerCounter;
    boolean balanceDone = false;

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
            timerCounter = 0;
            if (ypr[2] > balanceTarget) {
                // move backwards
                // slower the more times swapped
                // set motor * 1/2 * (timesSwappedCounter - 1)
                dt.xDriveTarget = 0.25 * Math.pow(0.5, (timesSwappedCounter));
            }
            if (ypr[2] < balanceTarget) {
                // move forwards
                // slower the more times swapped
                dt.xDriveTarget = -0.25 * Math.pow(0.5, (timesSwappedCounter));
            }
        } 
        timesSwappedCounter++;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
        System.out.println("Balancing Done");
    }
}
