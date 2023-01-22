// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
public class LimelightVisionSubsystem extends SubsystemBase {
  Double[] bp;
  /** Creates a new LimelightVisionSubsystem. */
  public LimelightVisionSubsystem() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //botpose is an array of 6 doubles, translation xyz then rotation xyz
    bp=NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new Double[] {});
    // if (bp.length>0){
    //   System.out.println(bp[0]+","+bp[1]+","+bp[2]+","+bp[3]+","+bp[4]+","+bp[5]);
    
    updateWidgets();
    }
  }

  
  private final ShuffleboardTab mainTab=Shuffleboard.getTab("Main");
  private final GenericPublisher mainBotPose=mainTab.add("botpose","").getEntry();
  private void updateWidgets(){
    if(bp.length>0)
    mainBotPose.setString(bp[0]+","+bp[1]+","+bp[2]+","+bp[3]+","+bp[4]+","+bp[5]);
  }
}
