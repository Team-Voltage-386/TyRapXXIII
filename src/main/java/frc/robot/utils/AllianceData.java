package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceData {
    
    public static Alliance m_Alliance = Alliance.Red;
    public static int joystickOrientationMultiplier = -1;
    public static double resetOrientationOffset = 180;

    public static void defineOrientations() {
        m_Alliance = DriverStation.getAlliance();
        if (m_Alliance == Alliance.Red) {
            joystickOrientationMultiplier = 1;
            resetOrientationOffset = 180;
        } else { //Blue
            joystickOrientationMultiplier = -1;
            resetOrientationOffset = 0;
        }
    }
}
