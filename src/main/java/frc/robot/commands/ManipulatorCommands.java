// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.utils.Flags.*;
import frc.robot.utils.PID;
import frc.robot.utils.PIDShufflable;
import frc.robot.utils.apriltag;

import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.AutoPilotConstants.*;
import static frc.robot.Constants.Limelightconstants.*;

import static frc.robot.Constants.ControllerConstants.*;

public class ManipulatorCommands extends CommandBase {
  private Drivetrain driveTrain;
  private Arm arm;
  private double targetX,targetY;
  private Limelight limelight;
  private apriltag currentATTarget=null;
  private double adjustableXDist;
  private double solidXDist;
  private double kYOffsetToScore=0.7;
  private double facingtoscore;
  private double deadband=0.1;
  private boolean povHeld;
  private PIDShufflable autoRPID;
  private PIDShufflable autoXPID;
  private PIDShufflable autoYPID;
  private boolean trafficConeMode;
  private boolean targetIsTop;
  private boolean targetIsLeft;


  /** Creates a new ManipulatorCommands. */
  public ManipulatorCommands(Arm ARM, Limelight LL,Drivetrain DT) {
    
    arm=ARM;
    limelight = LL;
    driveTrain=DT;
    adjustableXDist=6.25;//-6.25 if on blue team
    solidXDist=6.5;
    facingtoscore=-180;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    // addRequirements(limelight);
    // addRequirements(driveTrain);


    autoRPID=new PIDShufflable(kAutoRotationPID[0],kAutoRotationPID[1],kAutoRotationPID[2],"homeInRPID");
    autoXPID=new PIDShufflable(kAutoDriveXPID[0],kAutoDriveXPID[1],kAutoDriveXPID[2],"homeinXPID");
    autoYPID=new PIDShufflable(kAutoDriveYPID[0],kAutoDriveYPID[1],kAutoDriveYPID[2],"homeInYPID");

    //defaultStates
    trafficConeMode=false;//whatever we like
    targetIsLeft=true;
    targetIsTop=true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetX=0.0;targetY=0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // targetX+=kManipulator.getRawAxis(kLeftHorizontal)*.1;
    // targetY+=kManipulator.getRawAxis(kLeftVertical)*.1;
    //increment targets
    // if(kManipulator.getRawButtonPressed(kA))arm.ShoulderTarget-=15;
    // if(kManipulator.getRawButtonPressed(kY))arm.ShoulderTarget+=15;
    // if(kManipulator.getRawButtonPressed(kB))arm.ElbowTarget-=15;
    // if(kManipulator.getRawButtonPressed(kX))arm.ElbowTarget+=15;
    // arm.ArmIKDrive(targetX, targetY);
    // arm.JoystickDriveRawArm(-kManipulator.getRawAxis(kRightVertical), -kManipulator.getRawAxis(kLeftVertical));//uncomment this to just drive motors using joysticks
    updateSelectedTarget();

    //align to apriltag
    // if(limelight.apriltagmode() && Math.abs(kManipulator.getRawAxis(kLeftTrigger))>deadband&&limelight.apriltagsAvailable()){
    //   HumanDriverControl=false;
    //   if(target==null) {target=closestGrid(limelight.getPose()[0],limelight.getPose()[1]);}
    //   driveTrain.setFO(limelight.limelightYawToDriveTrainYaw());
    //   driveToTarget(adjustableXDist,target.y,facingtoscore,limelight.getPose()[0],limelight.getPose()[1],driveTrain.getProcessedHeading());
    // } 
    // //align to retroreflective
    // if(limelight.retroreflectivemode() && Math.abs(kManipulator.getRawAxis(kLeftTrigger))>deadband){
    //     HumanDriverControl=false;
    //     driveToTarget(0, 0, facingtoscore, 0, 1.5*Math.atan(Math.toRadians(limelight.tx())) , driveTrain.getProcessedHeading());//the value tiimes arctan should be a constant (variable of sorts)
    // } 
    // if(kManipulator.getRawButtonPressed(kX)){//switch pipelines
    //     if(limelight.apriltagmode())limelight.setPipeline(retroreflectivepipelineindex);
    //     else if(limelight.retroreflectivemode())limelight.setPipeline(apriltagpipelineindex);
    // }
    if(Math.abs(kManipulator.getRawAxis(kLeftTrigger))>.1){
      HumanDriverControl=false;
      lockOn();
    } else{
      HumanDriverControl=true;
      limelight.setPipeline(apriltagpipelineindex);
    }
    updateWidgets();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  //shuffleupdate
  public void updatePIDs(){
    if(autoXPID.detectChange()){autoXPID.shuffleUpdatePID();}
    if(autoYPID.detectChange()){autoYPID.shuffleUpdatePID();}
    if(autoRPID.detectChange()){autoRPID.shuffleUpdatePID();}

  }
  
  public void lockOn(){
    double pvx=0;
    double pvy=0;
    double pvr=0;
    //align to apriltag using apriltag
    if(!limelight.apriltagsAvailable()){currentATTarget=null;}
    if(limelight.apriltagmode() &&limelight.apriltagsAvailable()&&!trafficConeMode){
      HumanDriverControl=false;
      if(currentATTarget==null) {currentATTarget=closestGrid(limelight.getPose()[0],limelight.getPose()[1]);}
      driveTrain.setFO(limelight.limelightYawToDriveTrainYaw());
      // driveToTarget(adjustableXDist,target.y,facingtoscore,limelight.getPose()[0],limelight.getPose()[1],driveTrain.getProcessedHeading());
      pvx=adjustableXDist-limelight.getPose()[0];
      pvy=currentATTarget.y-limelight.getPose()[0];
      pvr=facingtoscore-driveTrain.getProcessedHeading();
    } 
    //align to retroreflective using retroreflective
    if(limelight.retroreflectivemode() && trafficConeMode){
        HumanDriverControl=false;
        driveToTarget(0, 0, facingtoscore, 0, 1.5*Math.atan(Math.toRadians(limelight.tx())) , driveTrain.getProcessedHeading());//the value tiimes arctan should be a constant (variable of sorts)
    }
    //align to retroreflective using apriltags?
    driveToTarget(pvx,pvy,pvr);
    widgetPVX.setDouble(pvx);
    widgetPVY.setDouble(pvy);
    widgetPVR.setDouble(pvr);
  }


  //utility;neccesary
  public void driveToTarget(double targetX, double targetY, double targetRot, double bpx, double bpy, double bpr){
    driveTrain.rotationTarget=autoRPID.calc(targetRot-bpr);
    driveTrain.xDriveTarget=autoXPID.calc(targetX-bpx);
    driveTrain.yDriveTarget=autoYPID.calc(targetY-bpy);
  }
  public void driveToTarget(double pvx,double pvy,double pvr){
    driveTrain.rotationTarget=autoRPID.calc(pvr);
    driveTrain.xDriveTarget=autoXPID.calc(pvx);
    driveTrain.yDriveTarget=autoYPID.calc(pvy);
  }
  public void updateSelectedTarget(){
    if(kManipulator.getRawButtonPressed(kLeftBumper)){
      if(trafficConeMode){trafficConeMode=false;}
      else if(!trafficConeMode){trafficConeMode=true;}
    }    
    if(!povHeld){
      if(kManipulator.getPOV()==0){
        targetIsTop=true;
      } if(kManipulator.getPOV()==180){
        targetIsTop=false;
      }
      if(trafficConeMode){
        if(kManipulator.getPOV()==90){
          targetIsLeft=false;
        }
        if(kManipulator.getPOV()==270){
          targetIsLeft=true;
        }
      }
    }
    povHeld=!(kManipulator.getPOV()==-1);
  }

  private final ShuffleboardTab m_tab=Shuffleboard.getTab("targeting");
  private final GenericEntry widgetTopLeft=m_tab.add("t1",false).withPosition(0, 0).getEntry();
  private final GenericEntry widgetTopMid=m_tab.add("t2",false).withPosition(1, 0).getEntry();
  private final GenericEntry widgetTopRight=m_tab.add("t3",false).withPosition(2, 0).getEntry();
  private final GenericEntry widgetMiddleLeft=m_tab.add("m1",false).withPosition(0, 1).getEntry();
  private final GenericEntry widgetMiddleMid=m_tab.add("m2",false).withPosition(1, 1).getEntry();
  private final GenericEntry widgetMiddleRight=m_tab.add("m3",false).withPosition(2, 1).getEntry();
  private final GenericEntry widgetPVX=m_tab.add("pvx",0.0).withPosition(0, 3).getEntry();
  private final GenericEntry widgetPVY=m_tab.add("pvy",0.0).withPosition(1, 3).getEntry();
  private final GenericEntry widgetPVR=m_tab.add("pvr",0.0).withPosition(2, 3).getEntry();
  private void updateWidgets(){
    widgetTopLeft.setBoolean(trafficConeMode&& targetIsLeft&&targetIsTop);
    widgetTopMid.setBoolean(!trafficConeMode&&targetIsTop);
    widgetTopRight.setBoolean(trafficConeMode&& !targetIsLeft && targetIsTop);
    widgetMiddleLeft.setBoolean(trafficConeMode&& targetIsLeft&& !targetIsTop);
    widgetMiddleMid.setBoolean(!trafficConeMode&& !targetIsTop);
    widgetMiddleRight.setBoolean(trafficConeMode&& !targetIsLeft&& !targetIsTop);
  }
}
