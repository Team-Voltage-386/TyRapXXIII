package frc.robot.utils;

public class AFFFinal extends PID {
    public double f;// f is maximum torque the motor fights
    public double s;// static force to overcome from feedforward

    public AFFFinal(double P, double I, double D, double F, double S) {
        super(P,I,D);
        f=F;
        s=S;
    }

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
    public double calc(double pv,double SpatialAngle, double extraload) {
        double result = super.calc(pv) + f * Math.cos(Math.toRadians(SpatialAngle)) + Math.signum(pv) * s + extraload;
        return result;
    }
    public double calc(double pv,double SpatialAngle){
        return calc(pv,SpatialAngle,0);
    }
}
