package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//where to put the flags that will be accessed by multiple subsystems
public class Flags {

    public static boolean IntakeDirection;// put out or suck

    public static boolean ConeMode;
    public static boolean canRotate = true;
    public static boolean scoreHigh;
    public static boolean armIsAtTarget;

}
