package frc.robot.utils;

public class PID {
    public double p;
    public double i;
    public double d;

    public double integralAcc;

    public double lastPV;

    protected long lastTime = 0;

    public PID(double P, double I, double D) {
        p = P;
        i = I;
        d = D;
        lastTime = System.currentTimeMillis();
    }

    public void reset() {
        integralAcc = 0;
        lastPV = 0;
    }

    public double calc(double pv) {
        long time = System.currentTimeMillis();
        double timeStep = (time - lastTime);
        timeStep /= 1000;
        if (timeStep > 0.5)
            timeStep = 0;
        integralAcc += pv * timeStep;
        lastTime = time;
        double result = (pv * p) + (integralAcc * i) - (((pv - lastPV) * timeStep) * d);
        lastPV = pv;
        return result;
    }
}
