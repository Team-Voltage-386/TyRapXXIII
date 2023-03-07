package frc.robot.utils;

import java.security.KeyStore.Entry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AFFShufflable extends PIDShufflable {
    public double f;// f is maximum torque the motor fights
    public double s;// static force to overcome from feedforward
    private double[] lastVals = new double[5];
    private boolean ready = false;

    public double load;// total load on this arm segment

    // this next block of stuff is just shuffleboard Implementation
    private static GenericSubscriber fUpdater;
    private static GenericSubscriber sUpdater;

    public AFFShufflable(double P, double I, double D, double F, double S, String EntryName) {
        super(P, I, D, EntryName);
        f = F;
        s = S;
        fUpdater = super.pidTab.addPersistent(EntryName + "fg", f).withPosition(super.pidObjectCount - 1, 3).getEntry();
        sUpdater = super.pidTab.addPersistent(EntryName + "sf", s).withPosition(super.pidObjectCount - 1, 4).getEntry();
        shuffleUpdatePID();

        pte = Shuffleboard.getTab(EntryName).add("pt", 0).getEntry();
        ite = Shuffleboard.getTab(EntryName).add("it", 0).getEntry();
        dte = Shuffleboard.getTab(EntryName).add("dt", 0).getEntry();
    }

    public AFFShufflable(double P, double I, double D, double F, double S, String EntryName, String TabName) {
        super(P, I, D, EntryName, TabName);
        f = F;
        s = S;

        fUpdater = super.pidTab.addPersistent(EntryName + "fg", f).withPosition(super.pidObjectCount - 1, 3).getEntry();
        sUpdater = super.pidTab.addPersistent(EntryName + "sf", s).withPosition(super.pidObjectCount - 1, 4).getEntry();
        shuffleUpdatePID();

        pte = Shuffleboard.getTab(EntryName).add("pt", 0).getEntry();
        ite = Shuffleboard.getTab(EntryName).add("it", 0).getEntry();
        dte = Shuffleboard.getTab(EntryName).add("dt", 0).getEntry();
    }

    // spatial angle is NOT neccesarily local angle, extra load is the torque load
    // from the other arm segment(s)
    /**
     * 
     * @param extraload    the load caused by not just the arm segment; example: the
     *                     shoulder supports the extraload of the elbow
     *                     (potentially)
     * @param SpatialAngle is the angle within real-world space; example: the
     *                     spatial elbow angle is different from encoder angle
     *                     because of
     *                     shoulder angle
     */
    public double calc(double pv, double SpatialAngle, double extraload) {

        double result = super.calc(pv) + f * Math.cos(Math.toRadians(SpatialAngle)) + Math.signum(pv) * s + extraload;
        double pt = pv * p;
        double it = integralAcc * i;
        double dt = -(((pv - lastPV) * timeStep) * d);
        updateVals(pt, it, dt);
        return result;
    }

    /**
     * (default calc, no extra load)
     * 
     * @param SpatialAngle is the angle within real-world space; example: the
     *                     spatial elbow angle is different from encoder angle
     *                     because of
     *                     shoulder angle
     */
    public double calc(double pv, double SpatialAngle) {
        return calc(pv, SpatialAngle, 0.0);
    }

    public double[] shiftOne(double insertion, double[] original) {
        double[] result = new double[original.length];
        result[0] = insertion;
        for (int i = 1; i < result.length; i++) {
            result[i] = original[i - 1];
        }
        return result;
    }

    public double averageArray(double[] input) {
        double sum = 0;
        for (double i : input) {
            sum += i;
        }
        return sum / ((double) input.length);
    }

    public double getLoad() {
        return load;
    }

    public void shuffleUpdatePID() {
        super.shuffleUpdatePID();
        f = fUpdater.getDouble(f);
        s = sUpdater.getDouble(s);
    }

    private GenericEntry pte = null;
    private GenericEntry ite = null;
    private GenericEntry dte = null;

    public void updateVals(double p, double i, double d) {
        if (pte != null)
            pte.setDouble(p);
        if (ite != null)
            ite.setDouble(i);
        if (dte != null)
            dte.setDouble(d);
    }

    public boolean detectChange() {
        return super.detectChange() || f != fUpdater.getDouble(f) || s != sUpdater.getDouble(s);
    }
}
