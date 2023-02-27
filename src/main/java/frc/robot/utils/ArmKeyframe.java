package frc.robot.utils;

public class ArmKeyframe {
    public static enum flaggingStates {
        stowed, score, pickup
    }
    
    public double[] keyFrameAngles;
    public flaggingStates keyFrameState;
    public ArmKeyframe(double[] angles, flaggingStates state){
        keyFrameAngles=angles;
        keyFrameState=state;
    }
    public double[] getKeyFrameAngles(){
        return keyFrameAngles;
    }
}
