package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDShufflable extends PID {
    public String name;

    protected ShuffleboardTab pidTab;
    private GenericSubscriber pUpdater;
    private GenericSubscriber iUpdater;
    private GenericSubscriber dUpdater;
    public static int pidObjectCount = 0;

    public PIDShufflable(double P, double I, double D, String PIDName) {
        super(P, I, D);
        lastTime = System.currentTimeMillis();

        pidTab = Shuffleboard.getTab("PIDTuning");
        name = PIDName;
        pUpdater = pidTab.addPersistent(PIDName + "_p", p).withPosition(pidObjectCount, 0).withSize(1, 1).getEntry();
        iUpdater = pidTab.addPersistent(PIDName + "_i", i).withPosition(pidObjectCount, 1).withSize(1, 1).getEntry();
        dUpdater = pidTab.addPersistent(PIDName + "_d", d).withPosition(pidObjectCount, 2).withSize(1, 1).getEntry();
        pidObjectCount++;
    }
    public PIDShufflable(double P, double I, double D, String PIDName, String TabName){
        super(P, I, D);
        lastTime = System.currentTimeMillis();
        pidTab = Shuffleboard.getTab(TabName);
        name = PIDName;
        pUpdater = pidTab.addPersistent(PIDName + "_p", p).withPosition(pidObjectCount, 0).withSize(1, 1).getEntry();
        iUpdater = pidTab.addPersistent(PIDName + "_i", i).withPosition(pidObjectCount, 1).withSize(1, 1).getEntry();
        dUpdater = pidTab.addPersistent(PIDName + "_d", d).withPosition(pidObjectCount, 2).withSize(1, 1).getEntry();
        pidObjectCount++;

    }

    public void shuffleUpdatePID() {
        super.p = pUpdater.getDouble(super.p);
        super.i = iUpdater.getDouble(super.i);
        super.d = dUpdater.getDouble(super.d);
        super.reset();
    }

    @Override
    public void reset() {
        super.reset();
    }

    public boolean detectChange() {
        return p != pUpdater.getDouble(p) || i != iUpdater.getDouble(i) || d != dUpdater.getDouble(d);
    }
}
