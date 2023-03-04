package frc.robot.utils;

public class ArmKeyframe {
    public static enum armKeyFrameStates {
        stowed, score, pickup, intermediary
    }

    public double[] keyFrameAngles;
    public armKeyFrameStates keyFrameState;

    public ArmKeyframe(double[] angles, armKeyFrameStates state) {
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
