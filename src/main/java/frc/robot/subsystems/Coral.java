package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LimelightConstants.*;

import java.lang.Math;

public class Coral extends SubsystemBase {
    private NetworkTableInstance inst;
    private NetworkTable table;
    private NetworkTableEntry pipeline;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    public Coral() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("limelight");
        pipeline = table.getEntry("getpipe");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        table.getEntry("pipeline").setNumber(0);
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    public double getTy() {
        return ty.getDouble(0.0);
    }

    public double getTa() {
        return ta.getDouble(0.0);
    }

    public long getPipeline() {
        return pipeline.getInteger(0);
    }

    public double getDistanceMeters() {
        double angle = (90 - kLimelightDegreesFromVertical) + getTy();
        double angleToRadians = Math.toRadians(angle);
        return Math.tan(angleToRadians) * kLimelightHeightMeters;
    }

    @Override
    public void periodic() {
        System.out.println("LL Distance = " + getDistanceMeters());
    }
}
