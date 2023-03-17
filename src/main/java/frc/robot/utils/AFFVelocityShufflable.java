package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AFFVelocityShufflable extends PID {
    public String name;

    protected ShuffleboardTab pidTab;
    private GenericSubscriber pUpdater;
    private GenericSubscriber iUpdater;
    private GenericSubscriber dUpdater;
    private GenericSubscriber fUpdater;
    public double f;
    public double velocityD;

    public static int pidObjectCount = 0;

    public AFFVelocityShufflable(double P, double I, double D, double F, String PIDName, String TabName) {
        super(P, I, 0);
        velocityD=D;
        f=F;
        pidTab = Shuffleboard.getTab(TabName);
        name = PIDName;
        pUpdater = pidTab.addPersistent(PIDName + "_p", p).withPosition(0, pidObjectCount).withSize(1, 1).getEntry();
        iUpdater = pidTab.addPersistent(PIDName + "_i", i).withPosition(1, pidObjectCount).withSize(1, 1).getEntry();
        dUpdater = pidTab.addPersistent(PIDName + "_d", velocityD).withPosition(2, pidObjectCount).withSize(1, 1).getEntry();
        fUpdater = pidTab.addPersistent(PIDName+"_f",f).withPosition(3, pidObjectCount).withSize(1, 1).getEntry();
        pidObjectCount++;

    }

    public void shuffleUpdatePID() {
        super.p = pUpdater.getDouble(super.p);
        super.i = iUpdater.getDouble(super.i);
        velocityD = dUpdater.getDouble(velocityD);
        f=fUpdater.getDouble(f);

        super.reset();
    }

    @Override
    public void reset() {
        super.reset();
    }

    public boolean detectChange() {
        return p != pUpdater.getDouble(p) || i != iUpdater.getDouble(i) || velocityD != dUpdater.getDouble(velocityD)||f!=fUpdater.getDouble(f);
    }

    /**
     * 
     * @param pv degrees of error
     * @param velocityPV degrees/sec (i think)
     * @param spatialAngle in degrees
     * @return
     */
    public double calc(double pv, double velocityPV, double spatialAngle){
        return super.calc(pv)+velocityD*velocityPV+f*Math.cos(Math.toRadians(spatialAngle));
    }
}
