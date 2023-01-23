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
  private boolean apriltagmode;
  private boolean retroreflectivemode;
  private NetworkTable nt;
  /** Creates a new LimelightVisionSubsystem. */
  public Limelight() {
    nt=NetworkTableInstance.getDefault().getTable("limelight");
  }
  /**alternate constructor for Limelight*/
  public Limelight(String networkTableName){
    nt=NetworkTableInstance.getDefault().getTable(networkTableName);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //botpose is an array of 6 doubles, translation xyz then rotation xyz
    apriltagmode= nt.getEntry("getpipe").getInteger(-1)==apriltagpipelineindex;
    retroreflectivemode=nt.getEntry("getpipe").getInteger(-1)==retroreflectivepipelineindex;
    if(apriltagmode){
      bp=nt.getEntry("botpose").getDoubleArray(new Double[] {});
    }

    updateWidgets();
  }
  public Double[] getPose(){
    return bp;
  }
  public boolean tagsAvailable(){
    return bp.length>0;
  }
  public void setPipeline(int index){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(index);
  }
  //not needed
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private final GenericPublisher mainBotPose=mainTab.add("botpose","").getEntry();

  private void updateWidgets(){
    if(bp.length>0)
    mainBotPose.setString(bp[0]+","+bp[1]+","+bp[2]+","+bp[3]+","+bp[4]+","+bp[5]);
  }
}
