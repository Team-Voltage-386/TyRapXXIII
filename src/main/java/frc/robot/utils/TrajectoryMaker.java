package frc.robot.utils;

public class TrajectoryMaker {
    public double a3, a2, tstep;
    public int divisions;
    public double[] trajectoryPoints;

    public TrajectoryMaker(int Divisions) {
        divisions = Divisions;
    }

    public double[] generateTrajectory(double InitialValue, double targetValue) {
        a2 = 3.0 * (targetValue - InitialValue); // Juan's math defines tf=arbitrary value we define; alternatively we can use
                                       // tf=1 and do 1/stepcount as an increment of step
        a3 = -2.0 / 3.0 * a2;
        trajectoryPoints = new double[divisions];
        tstep = 1.0 / (double) divisions;
        for (int i = 0; i < divisions; i++) {
            double ithT = i * tstep;
            trajectoryPoints[i] = a3 * Math.pow(ithT, 3) + a2 * Math.pow(ithT, 2) + InitialValue;
        }
        trajectoryPoints[divisions - 1] = targetValue;
        return trajectoryPoints;
    }
}