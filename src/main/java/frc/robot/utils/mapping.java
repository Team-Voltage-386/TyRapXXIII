package frc.robot.utils;

public class mapping {
    /**
     * take an input within the 'from' range, transpose it to the 'to' range
     * 
     * @param input
     * @param fromMin
     * @param fromMax
     * @param toMin
     * @param toMax
     * @return
     */
    public static double mapValue(double input, double fromMin, double fromMax, double toMin, double toMax) {
        return ((input - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin);
    }
}
