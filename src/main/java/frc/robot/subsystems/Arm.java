// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;
public class Arm extends SubsystemBase {
  public double ArmUpperAngle;
  public double ArmLowerAngle;
  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ArmIKDrive(double targetX, double targetY){//where x and y are relative to shoulder position
    double r=Math.sqrt(Math.pow(targetX, 2)+Math.pow(targetY, 2));
    ArmUpperAngle=(Math.acos(targetY));
  }
}
