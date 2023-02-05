package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;
import frc.robot.utils.PIDShufflable;

public class Balance extends CommandBase {
    private double balanceTarget = 2.5;
    private final Drivetrain dt;
    private Timer time = new Timer();
    private final PID pid = new PID(0.06, 0, 0.15);
    private boolean XLOCK = false;

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        time.reset();
        time.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("XLOCK", XLOCK);
        //assigns ypr vals
        SmartDashboard.putNumber("Pigeon Pitch", dt.ypr[2]);
        //balancing
        if(XLOCK == false)
        dt.xDriveTarget = -pid.calc(0 - dt.ypr[2]);
        else dt.xDriveTarget = 0;


        //BALANCE SYS
        SmartDashboard.putNumber("time in seconds", time.get());

        boolean isBalanced = false;

        if(Math.abs(dt.ypr[2]) < balanceTarget) {
            if(isBalanced == false) {
                isBalanced = true;
            }
        }
        else {
            time.reset();
            isBalanced = false;
        }

        if(time.get() > 0.15) XLOCK = true;
        else XLOCK = false;
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
