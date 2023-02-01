// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PID;

import static frc.robot.Constants.ArmConstants.*;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Arm extends SubsystemBase {
  private PID ArmUpperPID;
  private PID ArmLowerPID;
  public double ShoulderTarget;
  public double ElbowTarget;

  public double ArmUpperAngleOffset;
  public double ArmLowerAngleOffset;

  private TalonSRX ShoulderMotor;
  private Encoder ArmUpperEncoder;

  private TalonSRX ElbowMotor;
  private Encoder ArmLowerEncoder;
  /** Creates a new Arm. */
  public Arm() {
    ArmUpperPID=new PID(kArmUpperPID[0], kArmUpperPID[1], kArmUpperPID[2]);
    ArmLowerPID=new PID(kArmLowerPID[0], kArmLowerPID[1], kArmLowerPID[2]);
    ShoulderMotor=new TalonSRX(kArmUpperMotorID);
    ArmUpperEncoder=new Encoder(kArmUpperEncoderIDA, kArmUpperEncoderIDB);//, false, EncodingType.k4X)
    ElbowMotor=new TalonSRX(kArmLowerMotorID);
    ArmLowerEncoder = new Encoder(kArmLowerEncoderIDA, kArmLowerEncoderIDB);//, false, EncodingType.k4X
    ArmUpperAngleOffset=-90.0;
    ArmLowerAngleOffset=0.0;
    
    // ArmUpperEncoder.setReverseDirection(true);
    // ArmUpperMotor.setInverted(true);
    // ArmLowerEncoder.setReverseDirection(true);
    // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

    ArmUpperEncoder.setDistancePerPulse(kArmUpperEncoderConversion);
    ArmLowerEncoder.setDistancePerPulse(kArmLowerEncoderConversion);
    ShoulderTarget=ArmUpperAngleOffset;
    ElbowTarget=ArmLowerAngleOffset;
    ArmUpperEncoder.reset();
    ArmLowerEncoder.reset();
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setInverted(true);
    ElbowMotor.setInverted(true);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArmDrive();//move to target values
    // ArmUpperMotor.set(ControlMode.PercentOutput,1);
    // ArmLowerMotor.set(ControlMode.PercentOutput, 1);
    updateWidgets();
  }
  public double[] getArmAngles(){
    double[] result = {ArmUpperEncoder.getDistance()+ArmUpperAngleOffset,ArmLowerEncoder.getDistance()+ArmLowerAngleOffset};
    return result;
  }
  
  public void ArmDrive(){
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(kDrive(ShoulderTarget-getArmAngles()[0],true),getArmAngles()[0],kShoulderSafezone));
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(kDrive(ElbowTarget-getArmAngles()[1],false),getArmAngles()[1],kElbowSafezone));
  }
  public void JoystickDriveRawArm(double shoulder, double elbow){
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(kDrive(shoulder,true),getArmAngles()[0],kShoulderSafezone));
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,safeZoneDrive(kDrive(elbow,false),getArmAngles()[1],kElbowSafezone));
  }
  public void ArmDrive(double shoulder,double lower){
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,shoulder/100);
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,lower/100);
    ShoulderMotor.neutralOutput();
  }

  //filter PID and motor drive values to be within safe zone (PID goes in, safe number goes out)
  public double safeZoneDrive(double pv, double motorAngle, double[] safeZone){
    if(motorAngle>=safeZone[1]&&pv>0) return 0;
    if(motorAngle<=safeZone[0]&&pv<0) return 0;
    return pv;
  }
  //like PID but just constant drive
  public double kDrive(double pv,boolean shoulder){
    double drive;
    if(shoulder) drive=kArmShoulderMaxPercent; else drive=kArmElbowMaxPercent;
    if(pv<-kArmMotorDeadband)return -drive;//pv negative, drive positive
    if(pv>kArmMotorDeadband)return drive;//pv positive, drive negative
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
  private GenericPublisher armUpperAngleWidget=mainTab.add("upperArmAngle",0.0).withPosition(0,3).withSize(1,1).getEntry();
  private GenericPublisher armLowerAngleWidget=mainTab.add("lowerArmAngle",0.0).withPosition(1, 3).withSize(1, 1).getEntry();
  private GenericPublisher armUpperTargetWidget=mainTab.add("shouldertarget",0.0).withPosition(2, 3).withSize(1, 1).getEntry();
  private GenericPublisher armLowerTargetWidget=mainTab.add("elbowTarget",0.0).withPosition(3, 3).withSize(1, 1).getEntry();
  private GenericPublisher armUpperAngleRawWidget=mainTab.add("upperArmRaw",0.0).withPosition(4, 3).withSize(1, 1).getEntry();
  private GenericPublisher armLowerAngleRawWidget=mainTab.add("lowerArmRaw",0.0).withPosition(5, 3).withSize(1, 1).getEntry();
  private GenericPublisher armUpperDrivePercentWidget=mainTab.add("ShoulderDrive",0.0).withPosition(6, 3).withSize(1, 1).getEntry();
  private GenericPublisher armLowerDrivePercentWidget=mainTab.add("elbowDrive",0.0).withPosition(7, 3).withSize(1, 1).getEntry();

  
  public void updateWidgets(){
    armUpperAngleWidget.setDouble(getArmAngles()[0]);
    armLowerAngleWidget.setDouble(getArmAngles()[1]);
    armUpperTargetWidget.setDouble(ShoulderTarget);
    armLowerTargetWidget.setDouble(ElbowTarget);
    armUpperAngleRawWidget.setDouble(ArmUpperEncoder.getRaw());
    armLowerAngleRawWidget.setDouble(ArmLowerEncoder.getRaw());
    armUpperDrivePercentWidget.setDouble(safeZoneDrive(kDrive(ShoulderTarget-getArmAngles()[0],true),getArmAngles()[0],kShoulderSafezone));
    armLowerDrivePercentWidget.setDouble(safeZoneDrive(kDrive(ElbowTarget-getArmAngles()[1],false),getArmAngles()[1],kElbowSafezone));

  }
}
