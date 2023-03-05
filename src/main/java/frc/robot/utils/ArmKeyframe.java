package frc.robot.utils;

public class ArmKeyframe {
    public static enum armKeyFrameStates {
        stowed, score, pickup, intermediary, scoreCubeMid, scoreCubeHigh, scoreConeMid, scoreConeHigh
    }

    public double[] keyFrameAngles;
    public int substepsToHere;
    public armKeyFrameStates keyFrameState;

    public ArmKeyframe(double[] angles, armKeyFrameStates state, int substepsBeforeThis) {
        keyFrameAngles = angles;
        keyFrameState = state;
        substepsToHere = substepsBeforeThis;
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
            case scoreConeHigh:
                return "scoreConeHigh";
            case scoreConeMid:
                return "scoreConeMid";
            case scoreCubeHigh:
                return "scoreCubeHigh";
            case scoreCubeMid:
                return "scoreCubeMid";
        }
        return "null";
    }
}
