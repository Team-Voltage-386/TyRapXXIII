package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AFFShufflable extends PIDShufflable {
    public double f;// f is maximum torque the motor fights
    public double s;// static from feedforward
    public double load;// total load on this arm bit

    // this next block of stuff is just shuffleboard Implementation
    public String name;
    private static GenericSubscriber fUpdater;
    private static GenericSubscriber sUpdater;

    public AFFShufflable(double P, double I, double D, double F, double S, String TabName) {
        super(P, I, D, TabName);
        f = F;
        s = S;
        lastTime = System.currentTimeMillis();

        pidObjectCount++;
        super.pidTab = Shuffleboard.getTab(TabName);
        fUpdater = super.pidTab.addPersistent("f", f).withPosition(super.pidObjectCount, 3).getEntry();
        sUpdater = super.pidTab.addPersistent("s", s).withPosition(super.pidObjectCount, 4).getEntry();
        shuffleUpdatePID();
    }

    // spatial angle is NOT neccesarily local angle, extra load is the torque load
    // from the other arm segment(s)
    /**
     * 
     * @param extraload the load caused by not just the arm segment; example: the
     *                  shoulder supports the extraload of the elbow (potentially)
     * @return
     */
    public double calc(double pv, double SpatialAngle, double extraload) {
        double result = super.calc(pv) + f * Math.cos(Math.toRadians(SpatialAngle)) + extraload;
        super.lastPV = pv;
        return result;
    }

    /**
     * @param SpatialAngle is the angle within real-world space; example: the
     *                     shoulder
     * @return
     */
    public double calc(double pv, double SpatialAngle) {
        return calc(pv, SpatialAngle, 0.0);
    }

    public double getLoad() {
        return load;
    }

    public void shuffleUpdatePID() {
        super.shuffleUpdatePID();
        f = fUpdater.getDouble(f);
        s = sUpdater.getDouble(s);
        super.reset();
    }

    public boolean detectChange() {
        return super.detectChange() || f != fUpdater.getDouble(f) || s != sUpdater.getDouble(s);
    }
}
