package frc.robot.utils;

import com.ctre.phoenix.motion.TrajectoryPoint;

public class TrajectoryMaker {
    public static double a3, a2, tstep;

    public static double[] generateTrajectory(double InitialValue, double targetValue, int steps) {
        a2 = 3.0 * (targetValue - InitialValue); // Juan's math defines tf=arbitrary value we define; alternatively we
                                                 // can use
        // tf=1 and do 1/stepcount as an increment of step
        a3 = -2.0 / 3.0 * a2;
        double[] trajectoryPoints = new double[steps];// output trajectory points (sequence)
        tstep = 1.0 / (double) steps;
        for (int i = 0; i < steps; i++) {
            double ithT = i * tstep;
            trajectoryPoints[i] = a3 * Math.pow(ithT, 3) + a2 * Math.pow(ithT, 2) + InitialValue;
        }
        trajectoryPoints[steps - 1] = targetValue;
        return trajectoryPoints;
    }

    /**
     * 
     * @param initialPosition
     * @param targetPosition
     * @param steps
     * @return double arrays, array[0] is positions, array[1] is velocities
     */
    public static double[][] generatePositionVelocityTrajectories(double initialPosition, double targetPosition,
            int steps) {
        a2 = 3.0 * (targetPosition - initialPosition); // Juan's math defines tf=arbitrary value we define;
                                                       // alternatively we
        // can usetf=1 and do 1/stepcount as an increment of step
        a3 = -2.0 / 3.0 * a2;
        double[][] trajectories = new double[2][steps];
        for (int i = 0; i < steps; i++) {
            double ithT = i * tstep;
            trajectories[0][i] = a3 * Math.pow(ithT, 3) + a2 * Math.pow(ithT, 2) + initialPosition;
            trajectories[1][i] = 3 * a3 * Math.pow(ithT, 2) + 2 * a2 * ithT;// instant derivative of position(time)
        }
        trajectories[0][steps - 1] = targetPosition;
        trajectories[1][steps - 1] = 0;
        return trajectories;
    }

}
