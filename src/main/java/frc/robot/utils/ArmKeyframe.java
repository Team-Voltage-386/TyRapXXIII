package frc.robot.utils;

public class ArmKeyframe {
    public static enum flaggingStates {
        stowed, score, pickup, intermediary
    }

    public double[] keyFrameAngles;
    public flaggingStates keyFrameState;

    public ArmKeyframe(double[] angles, flaggingStates state) {
        keyFrameAngles = angles;
        keyFrameState = state;
    }

    public double[] getKeyFrameAngles() {
        return keyFrameAngles;
    }

    public String stateString() {
        switch (keyFrameState) {
            case stowed:
                return "stowed";
            case score:
                return "score";
            case pickup:
                return "pickup";
            case intermediary:
                return "intermediary";
        }
        return "null";
    }
}
