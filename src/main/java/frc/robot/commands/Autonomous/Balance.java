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
    private boolean isforward;
    private int directionFlipper;
    //shuffleboard stuff
    public static ShuffleboardTab mainTab = Shuffleboard.getTab("BalanceInfo");
    private static final GenericEntry xPosWidget = mainTab.add("X", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();
    private static final GenericEntry pitchWid = mainTab.add("Pitch",0).withPosition(0, 1).withSize(1,1).getEntry();
    private static final GenericEntry HasBeenBalancedWid = mainTab.add("HasBeenBalanced",false).withPosition(1,1).withSize(1,1).getEntry();
    private static final GenericEntry Timer = mainTab.add("Timer",0).withPosition(0,2).withSize(1,1).getEntry();

    public Balance(boolean ISFORWARD, Drivetrain DT) {
        isforward = ISFORWARD;
        dt = DT;

        if(isforward) directionFlipper = 1;
        else directionFlipper = -1;
    }

    //basically starts timer
    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        time.reset();
        time.start();
    }

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
        dt.xDriveTarget = 0.1*directionFlipper;
        else dt.xDriveTarget = -0.05*directionFlipper;

        //BALANCE SYSTEM
        boolean isBalanced = false;

        //sets timer to zero unless youre within balance target. essentially starting timer when youre balanced.
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
        return HasBeenBalanced && time.get() > 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Balancing done.");
        time.stop();
    }

}