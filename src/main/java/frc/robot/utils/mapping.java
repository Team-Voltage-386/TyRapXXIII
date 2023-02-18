package frc.robot.utils;

public class mapping {
    /**
     * 
     * @param input
     * @param fromMin
     * @param fromMax
     * @param toMin
     * @param toMax
     * @return
     */
    public static double mapValue(double input,double fromMin,double fromMax,double toMin,double toMax){
        return((input-fromMin)/(fromMax-fromMin)*(toMax-toMin)+toMin);
    }
}
