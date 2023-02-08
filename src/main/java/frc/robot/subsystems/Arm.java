// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AFFShufflable;
import frc.robot.utils.PID;

import static frc.robot.Constants.ArmConstants.*;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Arm extends SubsystemBase {
  public AFFShufflable ShoulderFeedForward;
  public AFFShufflable ElbowFeedForward;
  public double ShoulderTarget;
  public double ElbowTarget;

  public double ShoulderAngleOffset;
  public double ElbowAngleOffset;

  private TalonSRX ShoulderMotor;
  private Encoder ShoulderEncoder;

  private TalonSRX ElbowMotor;
  private Encoder ElbowEncoder;
  /** Creates a new Arm. */
  public Arm() {
    ShoulderMotor=new TalonSRX(kShoulderMotorID);
    ShoulderEncoder=new Encoder(kShoulderEncoderIDA, kShoulderEncoderIDB);//, false, EncodingType.k4X)
    ElbowMotor=new TalonSRX(kElbowMotorID);
    ElbowEncoder = new Encoder(kElbowEncoderIDA, kElbowEncoderIDB);//, false, EncodingType.k4X
    ShoulderAngleOffset=kShoulderOffset;
    ElbowAngleOffset=kElbowOffset;
    
    // ArmUpperEncoder.setReverseDirection(true);
    // ArmUpperMotor.setInverted(true);
    // ArmLowerEncoder.setReverseDirection(true);
    // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

    ShoulderEncoder.setDistancePerPulse(kArmUpperEncoderConversion);
    ElbowEncoder.setDistancePerPulse(kArmLowerEncoderConversion);
    ShoulderTarget=ShoulderAngleOffset;
    ElbowTarget=ElbowAngleOffset;
    ShoulderEncoder.reset();
    ElbowEncoder.reset();
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setInverted(true);
    ElbowMotor.setInverted(true);

    ShoulderFeedForward=new AFFShufflable(0, 0, 0, 0, 0, "ShoulderPID");
    ShoulderFeedForward.shuffleUpdatePID();
    ElbowFeedForward=new AFFShufflable(0, 0, 0, 0, 0, "ElbowPID");
    ElbowFeedForward.shuffleUpdatePID();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArmDrive();
    updateWidgets();
    fixPID();
  }
  public double[] getArmAngles(){
    double[] result = {ShoulderEncoder.getDistance()+ShoulderAngleOffset,ElbowEncoder.getDistance()+ElbowAngleOffset};
    return result;
  }
  
  //drive to target value always
  public void ArmDrive(){
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(ElbowFeedForward.calc(ElbowTarget-getArmAngles()[1],(getArmAngles()[0]+getArmAngles()[1])),getArmAngles()[1],kElbowSafezone));
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(ShoulderFeedForward.calc(ShoulderTarget-getArmAngles()[0],(getArmAngles()[0]),0*ElbowFeedForward.getLoad()),getArmAngles()[0],kShoulderSafezone));
  }

  //bang-bang to target value always
  public void ArmBangBang(){
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(bangbangdrive(ElbowTarget-getArmAngles()[1],kElbowMaxPercent), getArmAngles()[1], kElbowSafezone));
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(bangbangdrive(ShoulderTarget-getArmAngles()[0],kShoulderMaxPercent), getArmAngles()[0], kShoulderSafezone));
  }

  public void JoystickDriveRawArm(double shoulder, double elbow){
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(bangbangdrive(shoulder,kShoulderMaxPercent),getArmAngles()[0],kShoulderSafezone));
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(bangbangdrive(elbow,kElbowMaxPercent),getArmAngles()[1],kElbowSafezone));
  }

  //filter PID and motor drive values to be within safe zone (PID goes in, safe number goes out)
  public double safeZoneDrive(double pv, double motorAngle, double[] safeZone){
    if(motorAngle>=safeZone[1]&&pv>0) return 0;
    if(motorAngle<=safeZone[0]&&pv<0) return 0;
    return pv;
  }
  //like PID but just constant drive
  public double bangbangdrive(double pv,double motorMaxPercent){
    if(Math.abs(pv)>kArmMotorDeadband) return motorMaxPercent*Math.signum(pv);
    return 0;
  }

  //prerequisites: view the robot so that the arm extends to the right
    //when all arm angles are zeroed, the arm sticks straight out to the right
    //positive theta is counter clockwise, negative theta is clockwise
  public void ArmIKDrive(double targetX, double targetY,boolean stowable){//where x and y are relative to shoulder position //stow means it will stow nicely so by default TRUE
    double r=Math.sqrt(squareOf(targetY)+squareOf(targetX));
    if(r<kArmLowerLength+kArmUpperLength&&r>Math.abs(kArmLowerLength-kArmUpperLength)){//check for geometrically possible triangle
      double phi1=Math.acos((squareOf(kArmUpperLength)+squareOf(kArmLowerLength)-squareOf(r))/(2*kArmUpperLength*kArmLowerLength) );
      double phi2=Math.asin(kArmLowerLength*Math.sin(phi1)/r);
      //double phi3=Math.PI-(phi1+phi2);
        if(!stowable){
          ShoulderTarget=Math.toDegrees(phi1-Math.atan(targetY/targetX));
          ElbowTarget=Math.toDegrees(phi2-Math.PI);
        } else{
          ShoulderTarget=-Math.toDegrees(phi1-Math.atan(targetY/targetX));
          ElbowTarget=-Math.toDegrees(phi2-Math.PI);
        }
    }
  }
  

  //the default mode
  public void ArmIKDrive(double targetX, double targetY){
    ArmIKDrive(targetX, targetY, true);
  }

  private ShuffleboardTab mainTab=Shuffleboard.getTab("Main");
  private GenericPublisher shoulderAngleWidget=mainTab.add("ShoulderAngle",0.0).withPosition(0,3).withSize(1,1).getEntry();
  private GenericPublisher elbowAngleWidget=mainTab.add("elbowAngle",0.0).withPosition(1, 3).withSize(1, 1).getEntry();
  private GenericPublisher shoulderTargetWidget=mainTab.add("shouldertarget",0.0).withPosition(2, 3).withSize(1, 1).getEntry();
  private GenericPublisher elbowTargetWidget=mainTab.add("elbowTarget",0.0).withPosition(3, 3).withSize(1, 1).getEntry();
  private GenericPublisher shoulderRawWidget=mainTab.add("shoulderRaw",0.0).withPosition(4, 3).withSize(1, 1).getEntry();
  private GenericPublisher elbowRawWidget=mainTab.add("elbowRaw",0.0).withPosition(5, 3).withSize(1, 1).getEntry();
  private GenericPublisher shoulderDrivePercentageWidget=mainTab.add("ShoulderDrive",0.0).withPosition(6, 3).withSize(1, 1).getEntry();
  private GenericPublisher elbowDrivePercentWidget=mainTab.add("elbowDrive",0.0).withPosition(7, 3).withSize(1, 1).getEntry();

  
  public void updateWidgets(){
    shoulderAngleWidget.setDouble(getArmAngles()[0]);
    elbowAngleWidget.setDouble(getArmAngles()[1]);
    shoulderTargetWidget.setDouble(ShoulderTarget);
    elbowTargetWidget.setDouble(ElbowTarget);
    shoulderRawWidget.setDouble(ShoulderEncoder.getRaw());
    elbowRawWidget.setDouble(ElbowEncoder.getRaw());
    elbowDrivePercentWidget.setDouble(safeZoneDrive(ElbowFeedForward.calc(ElbowTarget-getArmAngles()[1],getArmAngles()[0]+getArmAngles()[1]),getArmAngles()[1],kElbowSafezone));
    shoulderDrivePercentageWidget.setDouble(safeZoneDrive(ShoulderFeedForward.calc(ShoulderTarget-getArmAngles()[0],getArmAngles()[0],0*ElbowFeedForward.getLoad()),getArmAngles()[0],kShoulderSafezone));
  }

  public void fixPID(){
    if(ElbowFeedForward.detectChange())ElbowFeedForward.shuffleUpdatePID();
    if(ShoulderFeedForward.detectChange())ShoulderFeedForward.shuffleUpdatePID();
  }
}
