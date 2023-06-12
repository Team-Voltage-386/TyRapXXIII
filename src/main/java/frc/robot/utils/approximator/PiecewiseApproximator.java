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
        if (Math.abs(x) < epsilon) {
            return 0;
        } else if (x > -phi && x < 0) {
            return 10;
        } else if (x < phi && x > 0) {
            return -10;
        } else {
            return m * x + b;
        }
    }

}
