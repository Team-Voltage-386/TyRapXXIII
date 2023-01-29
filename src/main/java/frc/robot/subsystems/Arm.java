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
  //prerequisites: view the robot so that the arm extends to the right
    //when all arm angles are zeroed, the arm sticks straight out to the right
    //positive theta is counter clockwise, negative theta is clockwise
  public void ArmIKDrive(double targetX, double targetY,boolean stowable){//where x and y are relative to shoulder position //stow means it will stow nicely so by default TRUE
    double r=Math.sqrt(squareOf(targetY)+squareOf(targetX));
    double phi1=Math.acos((squareOf(kArmUpperLength)+squareOf(kArmLowerLength)-squareOf(r))/(2*kArmUpperLength*kArmLowerLength) );
    double phi2=Math.asin(kArmLowerLength*Math.sin(phi1)/r);
    //double phi3=Math.PI-(phi1+phi2);
      if(!stowable){
        ArmUpperAngleTarget=Math.toDegrees(phi1-Math.atan(targetY/targetX));
        ArmLowerAngleTarget=Math.toDegrees(phi2-Math.PI);
      } else{
        ArmUpperAngleTarget=-Math.toDegrees(phi1-Math.atan(targetY/targetX));
        ArmLowerAngleTarget=-Math.toDegrees(phi2-Math.PI);
      }
  }
}
