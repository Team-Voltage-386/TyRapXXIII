package frc.robot.utils;

public class mapping {
    /**
     * take an input within the 'from' range, transpose it to the 'to' range. Note
     * that min and max are not neccesarily the actual range min and max
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

    /**
     * clamp the input to between min and max
     * 
     * @param input
     * @param min   MUST be range minimum
     * @param max   MUST be range maximum
     * @return input clamped
     *         APPARENTLY there is a MathUtils class by FRC that does this exact
     *         same thing and has the exact same code
     */
    public static double clamp(double input, double min, double max) {
        return Math.min(Math.max(input, min),max);
    }

}
