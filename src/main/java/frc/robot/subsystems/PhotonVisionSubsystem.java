// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.constantsHelpers.FieldTag;
import frc.robot.constantsHelpers.Grid;


public class PhotonVisionSubsystem extends SubsystemBase {
  public PhotonCamera camera;
  public PhotonPipelineResult result;
  public List<PhotonTrackedTarget> listOfTargets;
  
  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem(String CameraName) {
    camera = new PhotonCamera(CameraName);
    camera.setPipelineIndex(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    listOfTargets=result.getTargets();
  }

  public void setPipelineIndex(int i){
    camera.setPipelineIndex(i);
  }
  
  public boolean twoRows(){
    //check if it can be in the top or bottom section first
    //assuming we are in closest mode pipeline FOR BOTH APRIL TAG AND RETROREFLECTIVE MODE
    double minP = listOfTargets.get(0).getPitch();
    double maxP=listOfTargets.get(0).getPitch();
    double tp=0.0;
    if(listOfTargets.size()>2){
      for (PhotonTrackedTarget t : listOfTargets){
        tp=t.getPitch();
        if(tp>maxP) maxP=tp;
        if(tp<minP) minP=tp;
      } 
    }
    else if(listOfTargets.size()==2)
    {
      maxP=listOfTargets.get(1).getPitch();
    }
    else return false;

    return (listOfTargets.size()>1 || Math.abs(minP-maxP)<PhotonVisionConstants.AA);
  }


  //all these methods below are dependent on the apriltag pipeline being used
  //find matching targetting data
  public PhotonTrackedTarget getTargetData(FieldTag which){
    for(PhotonTrackedTarget i:listOfTargets){
      if (i.getFiducialId()==which.id) {
        return i;
      }
    }
    return null;//potential errors?
  }

  //find matching field data (physical constants)
  public FieldTag getFieldData(PhotonTrackedTarget which){
    for(Grid i : PhotonVisionConstants.grids){
      if (i.getTag().id==which.getFiducialId()){
        return i.getTag();
      }
    }
    return null;
  }

  public double ATr(FieldTag which){
    return PhotonVisionConstants.distAlg(which, getTargetData(which).getPitch());
  }

//unsure if these methods should be in a command with the odometry stuff or in this subsystem
  public double ATx(FieldTag which,double distance,double yaw){
    return which.x+distance*Math.cos(Math.PI/180 * (yaw + getTargetData(which).getYaw()));
  }

  public double ATy(FieldTag which,double distance,double yaw){
    return which.y+distance*Math.sin(Math.PI/180 * (yaw + getTargetData(which).getYaw()));
  }

  public double yawError(FieldTag one, FieldTag two, double y){
    double distOne=ATr(one);
    double distTwo=ATr(two);
    double yOne = getTargetData(one).getYaw();
    double yTwo = getTargetData(two).getYaw();
    return (yOne-yTwo) - Math.acos( Math.PI/180 * (Math.pow( ATy(one, distOne, yOne)-ATy(two,distTwo, yTwo),2) - Math.pow(distOne,2) - Math.pow(distTwo,2))/(2*distOne*distTwo) );
  }
}
