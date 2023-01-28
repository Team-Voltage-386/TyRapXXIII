// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;
public class Arm extends SubsystemBase {
  public double ArmUpperAngleTarget;
  public double ArmLowerAngleTarget;
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ArmIKDrive(double targetX, double targetY){//where x and y are relative to shoulder position
    double r=Math.sqrt(squareOf(targetY)+squareOf(targetX));
    //might need to add/subtract 180 + offset from encoder
    ArmUpperAngleTarget=Math.toDegrees(Math.acos((squareOf(r)+squareOf(kArmUpperLength)-squareOf(kArmLowerLength))/(2*kArmUpperLength*r))+Math.atan(targetY/targetX));
    ArmLowerAngleTarget=Math.toDegrees(Math.acos((squareOf(kArmUpperLength)+squareOf(kArmLowerLength)-squareOf(r))/(2*kArmLowerLength*kArmUpperLength)));
  }
}
