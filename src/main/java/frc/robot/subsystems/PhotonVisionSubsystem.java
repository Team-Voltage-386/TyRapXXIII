// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
  public PhotonCamera camera;
  public PhotonPipelineResult result;
  public List<PhotonTrackedTarget> list;
  
  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem(String CameraName) {
    camera = new PhotonCamera(CameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
  }
}
