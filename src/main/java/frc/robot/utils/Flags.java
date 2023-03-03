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
    public static boolean canRotate;
    public static boolean scoreHigh;

    public static ShuffleboardTab flagTab = Shuffleboard.getTab("Flags");
    public static GenericEntry ConeModeWidget = flagTab.add("coneMode", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#9900FF")).getEntry();
    public static GenericEntry scoreHighWidget = flagTab.add("scoreHigh", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFFFFF", "Color when false", "#999999")).getEntry();
    public static void updateWidgets(){
        ConeModeWidget.setBoolean(ConeMode);
        scoreHighWidget.setBoolean(scoreHigh);
    }
}
