package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AFFShufflable {
    public double p;//proper p now
    public double i;
    public double d;
    public double f;//f is maximum torque the motor fights
    public double s;//static from feedforward
    public double integralAcc;
    public double load;//total load on this arm bit

    public double lastPV;

    private long lastTime = 0;

    //this next block of stuff is just shuffleboard Implementation
    public String name;
    private static ShuffleboardTab pidTab;
    private static GenericSubscriber pUpdater;
    private static GenericSubscriber iUpdater;
    private static GenericSubscriber dUpdater;
    private static GenericSubscriber fUpdater;
    private static GenericSubscriber sUpdater;
    public static int pidObjectCount=0;


    
    public AFFShufflable(double P, double I, double D, double F, double S, String TabName){
        p = P;
        i = I;
        d = D;
        f=F;
        s=S;
        lastTime = System.currentTimeMillis();

        pidObjectCount++;
        pidTab=Shuffleboard.getTab(TabName);
        pUpdater=pidTab.addPersistent("P", p).withPosition(0,0).getEntry();
        iUpdater=pidTab.addPersistent("i", i).withPosition(1,0).getEntry();
        dUpdater=pidTab.addPersistent("d", d).withPosition(2,0).getEntry();
        fUpdater=pidTab.addPersistent("f", f).withPosition(3,0).getEntry();
        sUpdater=pidTab.addPersistent("s", s).withPosition(4, 0).getEntry();
        shuffleUpdatePID();
    }
    

    public void reset() {
        integralAcc = 0;
        lastPV = 0;
    }

    //spatial angle is NOT neccesarily local angle, extra load is the torque load from the other arm segment(s)
    public double calc(double pv,double SpatialAngle,double extraload) {
        long time = System.currentTimeMillis();
        double timeStep = (time - lastTime) / 1000;
        if (timeStep > 0.5)
            timeStep = 0;
        integralAcc += pv * timeStep;
        lastTime = time;
        loadUp(SpatialAngle, extraload);
        double result = p*pv + (s*Math.signum(pv)) + (integralAcc * i) - (((pv - lastPV) * timeStep) * d) + getLoad();
        lastPV = pv;
        return result;
    }

    public double calc(double pv, double SpatialAngle){
        return calc(pv, SpatialAngle,0.0);
    }

    public void loadUp(double SpatialAngle,double extraload){
        load= f*Math.cos(Math.toRadians(SpatialAngle))+extraload;
    }

    public double getLoad(){
        return load;
    }

    public void shuffleUpdatePID(){
        p=pUpdater.getDouble(p);
        i=iUpdater.getDouble(i);
        d=dUpdater.getDouble(d);
        f=fUpdater.getDouble(f);
        s=sUpdater.getDouble(s);
        reset(); 
    }

    public boolean detectChange(){
        return p!=pUpdater.getDouble(p)||i!=iUpdater.getDouble(i)||d!=dUpdater.getDouble(d)||f!=fUpdater.getDouble(f)||s!=sUpdater.getDouble(s);
    }
}
