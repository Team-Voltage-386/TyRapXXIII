package frc.robot.utils;

public class TrajectoryMaker {
    public static double a3, a2, tstep;

    public static double[] generateTrajectory(double InitialValue, double targetValue, int steps) {
        a2 = 3.0 * (targetValue - InitialValue); // Juan's math defines tf=arbitrary value we define; alternatively we can use
                                       // tf=1 and do 1/stepcount as an increment of step
        a3 = -2.0 / 3.0 * a2;
        double[] trajectoryPoints = new double[steps];//output trajectory points (sequence)
        tstep = 1.0 / (double) steps;
        for (int i = 0; i < steps; i++) {
            double ithT = i * tstep;
            trajectoryPoints[i] = a3 * Math.pow(ithT, 3) + a2 * Math.pow(ithT, 2) + InitialValue;
        }
        trajectoryPoints[steps - 1] = targetValue;
        return trajectoryPoints;
    }
}
