//This command can be deleted
package frc.robot.commands.Autonomous;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.AutoConstants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;

public class PIDBalance extends CommandBase {
    private Drivetrain dt;
    private final PID balancePID = new PID(0.03, 0, 0.1);
    private double ypr[] = new double[3];
    private Pigeon2 pigeon = new Pigeon2(kIMUid);

    private double balanceTarget = 2.5;

    public PIDBalance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pigeon.getYawPitchRoll(ypr);
        dt.xDriveTarget = balancePID.calc(0 - ypr[2]);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(ypr[2]) <= balanceTarget) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
        System.out.println("Balancing Done");
    }

}
