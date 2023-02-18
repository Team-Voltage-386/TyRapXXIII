package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import frc.robot.utils.PIDShufflable;

public class PIDBalance extends CommandBase {
    private final Drivetrain dt;

    private double ypr[] = new double[3];
    public Pigeon2 pigeon = new Pigeon2(kIMUid);

    // init vars and methods
    private double balanceTarget = 2.5;
    
    private Timer time = new Timer();

    private final PIDShufflable pid = new PIDShufflable(0.08, 1, 1.3, "BalanceInfo");

    // controls if the robot is Xlocked or not
    private boolean XLOCK = false;

    public PIDBalance(Drivetrain DT) {
        dt = DT;
    }

    // basically starts timer
    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    // FUN STUFF.
    @Override
    public void execute() {
        pigeon.getYawPitchRoll(ypr);

        // balancing if not xlock
        if (XLOCK == false)
            dt.xDriveTarget = -pid.calc(0 - ypr[2]);
        else
            dt.xDriveTarget = 0;

        // BALANCE SYSTEM
        boolean isBalanced = false;

        if (Math.abs(ypr[2]) < balanceTarget) {
            if (isBalanced == false) {
                isBalanced = true;
            }
        } else {
            time.reset();
            isBalanced = false;
        }

        if (time.get() > 0.2)
            XLOCK = true;
        else
            XLOCK = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Balancing done.");
        time.stop();
    }

}