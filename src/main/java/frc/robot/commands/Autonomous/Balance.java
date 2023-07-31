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

public class Balance extends CommandBase {
    // init vars and methods
    private double balanceTarget = 2;
    private final Drivetrain dt;
    private Timer time = new Timer();
    private final PIDShufflable pid = new PIDShufflable(0.03, 0, 1, "BalanceInfo");
    // controls if the robot is Xlocked or not
    private boolean XLOCK = false;
    // shuffleboard stuff
    public static ShuffleboardTab mainTab = Shuffleboard.getTab("BalanceInfo");
    private static final GenericEntry xPosWidget = mainTab.add("X", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();
    private static final GenericEntry pitchWid = mainTab.add("Pitch", 0).withPosition(0, 1).withSize(1, 1).getEntry();
    private static final GenericEntry XLOCKWid = mainTab.add("XLOCK", false).withPosition(1, 1).withSize(1, 1)
            .getEntry();
    private static final GenericEntry Timer = mainTab.add("Timer", 0).withPosition(0, 2).withSize(1, 1).getEntry();

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    // basically starts timer
    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        time.reset();
        time.start();
    }

    // FUN STUFF.
    @Override
    public void execute() {
        // update values 50 times a sec
        xPosWidget.setDouble(dt.xPos);
        yPosWidget.setDouble(dt.yPos);
        pitchWid.setDouble(dt.ypr[2]);
        XLOCKWid.setBoolean(XLOCK);
        Timer.setDouble(time.get());

        // balancing if not xlock
        if (!XLOCK)
            dt.xDriveTarget = -pid.calc(0 - dt.ypr[2]);
        else
            dt.xDriveTarget = 0;

        // BALANCE SYSTEM
        boolean isBalanced = false;

        if (Math.abs(dt.ypr[2]) <= balanceTarget) {
            if (!isBalanced) {
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
