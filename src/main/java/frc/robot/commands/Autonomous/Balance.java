package frc.robot.commands.Autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
    //init vars and methods
    private double balanceTarget = 4;
    private final Drivetrain dt;
    private Timer time = new Timer();
    //controls if the robot is HasBeenBalanceded or not
    private boolean HasBeenBalanced = false;
    //shuffleboard stuff
    public static ShuffleboardTab mainTab = Shuffleboard.getTab("BalanceInfo");
    private static final GenericEntry xPosWidget = mainTab.add("X", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();
    private static final GenericEntry pitchWid = mainTab.add("Pitch",0).withPosition(0, 1).withSize(1,1).getEntry();
    private static final GenericEntry HasBeenBalancedWid = mainTab.add("HasBeenBalanced",false).withPosition(1,1).withSize(1,1).getEntry();
    private static final GenericEntry Timer = mainTab.add("Timer",0).withPosition(0,2).withSize(1,1).getEntry();

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    //basically starts timer
    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        time.reset();
        time.start();
    }

    //FUN STUFF.
    @Override
    public void execute() {
        //update values 50 times a sec
        xPosWidget.setDouble(dt.xPos);
        yPosWidget.setDouble(dt.yPos);
        pitchWid.setDouble(dt.ypr[2]);
        HasBeenBalancedWid.setBoolean(HasBeenBalanced);
        Timer.setDouble(time.get());

        //balancing if not HasBeenBalanced
        if(!HasBeenBalanced)
        dt.xDriveTarget = 0.1;
        else dt.xDriveTarget = -0.05;

        //BALANCE SYSTEM
        boolean isBalanced = false;

        if(Math.abs(dt.ypr[2]) <= balanceTarget) {
            if(!isBalanced) {
                isBalanced = true;
            }
        }
        else {
            time.reset();
            isBalanced = false;
        }

        if(time.get() > 0.1) HasBeenBalanced = true;
        else HasBeenBalanced = false;
    }

    @Override
    public boolean isFinished() {
        return HasBeenBalanced && time.get() > 0.4;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Balancing done.");
        time.stop();
    }

}
//Lucas when you wake up and see this in the morning, check slack to refresh your mind on where you were at - Lucas' self note 3/19/23