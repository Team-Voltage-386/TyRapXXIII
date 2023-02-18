package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;import edu.wpi.first.wpilibj.shufflebaord.ShuffleboardTab;
import frc.robot.commands.Autonomous.LogicBalance;

public class PIDShufflable {
    public double p;
    public double i;
    public double d;
    public String name;
    public double integralAcc;

    public double lastPV;

    private long lastTime = 0;

    //Shuffleboard implementation
    private static GenericSubscriber pUpdater;
    private static GenericSubscriber iUpdater;
    private static GenericSubscriber dUpdater;
    public static int pidObjectCount = 0;

    public PIDShuffleable(double P, double I, double D, String TabName) {

    }
}
