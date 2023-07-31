package frc.robot.utils.approximator;

/**
 * Approximates a linear output given an input
 */
public class LinearApproximator implements Approximatable {

    private double m;
    private double b;

    public LinearApproximator(double M, double B) {
        this.m = M;
        this.b = B;
    }

    @Override
    public double approximate(double x) {
        return m * x + b;
    }

}
