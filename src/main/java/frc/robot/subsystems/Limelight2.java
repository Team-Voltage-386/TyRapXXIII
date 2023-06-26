package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import static frc.robot.Constants.Limelightconstants.*;

public class Limelight2 extends SubsystemBase {
    private Double[] bp;
    private Double tx, ty;
    private boolean apriltagmode;
    private boolean retroreflectivemode;
    private NetworkTable nt;

    public Limelight2() {
        nt = NetworkTableInstance.getDefault().getTable("limelight");
    }

    //alternate constructor
    public Limelight2(String networkTableName) {
        nt = NetworkTableInstance.getDefault().getTable(networkTableName);
    }

}

@Override
public void periodic() {
    //This method will be called once per scheduler run
    //botpose is an array of 6 doubles, translation xyz then rotation xyz
    apriltagmode = nt.getEntry("getpipe").getInteger(-1) == apriltagpipelineindex;
    retroreflectivemode = nt.getEntry("getpipe").getInteger(-1) == retroreflectivepipelineindex;
    if (apriltagmode) {
        bp = nt.getEntry("botpose").getDoubleArray(new Double[] {});
    }
}   
