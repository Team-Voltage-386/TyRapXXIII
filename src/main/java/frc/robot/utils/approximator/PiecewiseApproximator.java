package frc.robot.utils.approximator;

public class PiecewiseApproximator implements Approximatable{

    private double epsilon;
    private double phi;
    private double m;
    private double b;

    public PiecewiseApproximator(double EPSILON, double PHI, double M, double B) {
        this.epsilon = EPSILON;
        this.phi = PHI;
        this.m = M;
        this.b = B;
    }
    @Override
    public double approximate(double x) {
        if ()
        return 0;
    }

}
