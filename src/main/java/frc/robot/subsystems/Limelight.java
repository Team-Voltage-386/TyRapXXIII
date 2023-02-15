// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import static frc.robot.Constants.Limelightconstants.*;

public class Limelight extends SubsystemBase {
  private Double[] bp;
  private Double tx, ty;
  private boolean apriltagmode;
  private boolean retroreflectivemode;
  private NetworkTable nt;

  /** Creates a new LimelightVisionSubsystem. */
  public Limelight() {
    nt = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** alternate constructor for Limelight */
  public Limelight(String networkTableName) {
    nt = NetworkTableInstance.getDefault().getTable(networkTableName);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // botpose is an array of 6 doubles, translation xyz then rotation xyz
    apriltagmode = nt.getEntry("getpipe").getInteger(-1) == apriltagpipelineindex;
    retroreflectivemode = nt.getEntry("getpipe").getInteger(-1) == retroreflectivepipelineindex;
    if (apriltagmode) {
      bp = nt.getEntry("botpose").getDoubleArray(new Double[] {});
    }
    tx = nt.getEntry("tx").getDouble(0);
    ty = nt.getEntry("ty").getDouble(0);

    updateWidgets();
  }
  /**@return bot pose as
   * double array length six - 
   * x y z position, x y z rotation
   */
  public Double[] getPose() {
    return bp;
  }
  /**@return target x angle*/
  public double tx() {
    return tx;
  }
  /**@return target y angle*/
  public double ty() {
    return ty;
  }
  
  /**
   * @return are there april tags available? does check if pipeline is apriltag
   * 
   */
  public boolean apriltagsAvailable() {
    return apriltagmode && bp.length > 0;
  }

  /**@return are there targets regardless of pipeline */
  public boolean targetsAvailable() {
    return nt.getEntry("tv").getInteger(-1) == 1 || (apriltagmode && bp.length > 0);
  }
  
  public void setPipeline(int index) {
    nt.getEntry("pipeline").setNumber(index);
    nt.getEntry("ledMode").setNumber(0);
  }
  /**@return is pipeline apriltag */
  public boolean apriltagmode() {
    return apriltagmode;
  }
  /**@return is pipeline retroreflective */
  public boolean retroreflectivemode() {
    return retroreflectivemode;
  }

  /**
   * set limelight crop both x and y use values within -1 to 1
   * @param xMin
   * @param xMax
   * @param yMin
   * @param yMax
   */
  public void setCameraCrop(double xMin, double xMax, double yMin, double yMax){
    nt.getEntry("crop").setDoubleArray(new double[]{xMin,xMax,yMin,yMax});
  }
  /**
   * use double array of size four to set limelight cropping
   * @param cropValues
   */
  public void setCameraCrop(double[] cropValues){
    setCameraCrop(cropValues[0],cropValues[1],cropValues[2],cropValues[3]);
  }
  /**reset camera cropping */
  public void resetCameraCrop(){
    setCameraCrop(-1, 1, -1, 1);
  }

  // not needed
  private final ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight");
  private final GenericPublisher BotPoseWidget = limelightTab.add("botpose", "").withPosition(2, 1).withSize(4, 1)
      .getEntry();
  private final GenericPublisher HastargetsWidget = limelightTab.add("targets", false).withPosition(1, 1).withSize(1, 1)
      .getEntry();
  private final GenericPublisher PipelineWidget = limelightTab.add("pipelineIndex", 0).withPosition(0, 1).withSize(1, 1)
      .getEntry();

  private void updateWidgets() {
    HastargetsWidget.setBoolean(targetsAvailable());
    PipelineWidget.setInteger(nt.getEntry("getpipe").getInteger(-1));
    if (apriltagmode && bp.length > 0)
      BotPoseWidget.setString(bp[0] + "," + bp[1] + "," + bp[2] + "," + bp[3] + "," + bp[4] + "," + bp[5]);
  }
  public double limelightYawToDriveTrainYaw(){
    if(apriltagsAvailable())return (this.getPose()[5]+180-90);//limelight field coordinate system is 180, gyroscope weirdness says subtract 90
    else return 0;
  }
}
