package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//where to put the flags that will be accessed by multiple subsystems
public class Flags {

    // public static boolean IntakeDirection;// put out or suck
    public static boolean ConeMode;
    public static boolean handCanRotate;
    public static boolean scoreHighTarget;
    public static boolean armIsAtTarget;
    public static subsystemsStates manipulatorSetState;

    public static enum subsystemsStates {
        runStow, runScore, runPickup
    }
    public static String statesString(subsystemsStates state){
        switch(state){
            case runStow:
            return "runStow";
            case runScore:
            return "runScore";
            case runPickup:
            return "runPickup";
            default:
            return "bad";
        }
    }
}
