package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDShufflable {
    public double p;
    public double i;
    public double d;
    public String name;
    public double integralAcc;

    public double lastPV;

    private long lastTime = 0;

    // this next block of stuff is just shuffleboard Implementation
    private ShuffleboardTab pidTab;
    private GenericSubscriber pUpdater;
    private GenericSubscriber iUpdater;
    private GenericSubscriber dUpdater;
    public static int pidObjectCount = 0;

    public PIDShufflable(double P, double I, double D, String PIDName) {
        p = P;
        i = I;
        d = D;
        lastTime = System.currentTimeMillis();

        pidTab = Shuffleboard.getTab("PIDTuning");
        pUpdater = pidTab.addPersistent(PIDName+"_p", p).withPosition(pidObjectCount, 0).withSize(1, 1).getEntry();
        iUpdater = pidTab.addPersistent(PIDName+"_i", i).withPosition(pidObjectCount, 1).withSize(1, 1).getEntry();
        dUpdater = pidTab.addPersistent(PIDName+"_d", d).withPosition(pidObjectCount, 2).withSize(1, 1).getEntry();
        pidObjectCount++;
        shuffleUpdatePID();
    }

    public void reset() {
        integralAcc = 0;
        lastPV = 0;
    }

    public double calc(double pv) {
        long time = System.currentTimeMillis();
        double timeStep = (time - lastTime) / 1000;
        if (timeStep > 0.5)
            timeStep = 0;
        integralAcc += pv * timeStep;
        lastTime = time;
        double result = (pv * p) + (integralAcc * i) - (((pv - lastPV) * timeStep) * d);
        lastPV = pv;
        return result;
    }

    public void shuffleUpdatePID() {
        p = pUpdater.getDouble(p);
        i = iUpdater.getDouble(i);
        d = dUpdater.getDouble(d);
        reset();
    }

    public boolean detectChange() {
        return p != pUpdater.getDouble(p) || i != iUpdater.getDouble(i) || d != dUpdater.getDouble(d);
    }
}
