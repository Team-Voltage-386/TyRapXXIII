// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PID;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Arm extends SubsystemBase {
  private PID ArmUpperPID;
  private PID ArmLowerPID;
  public double ArmUpperAngleTarget;
  public double ArmLowerAngleTarget;

  public double ArmUpperAngleOffset;
  public double ArmLowerAngleOffset;

  private TalonSRX ArmUpperMotor;
  private Encoder ArmUpperEncoder;

  private TalonSRX ArmLowerMotor;
  private Encoder ArmLowerEncoder;
  /** Creates a new Arm. */
  public Arm() {
    ArmUpperPID=new PID(kArmUpperPID[0], kArmUpperPID[1], kArmUpperPID[2]);
    ArmLowerPID=new PID(kArmLowerPID[0], kArmLowerPID[1], kArmLowerPID[2]);
    ArmUpperMotor=new TalonSRX(kArmUpperMotorID);
    ArmUpperEncoder=new Encoder(kArmUpperEncoderIDA, kArmUpperEncoderIDB);
    ArmLowerMotor=new TalonSRX(kArmLowerMotorID);
    ArmLowerEncoder = new Encoder(kArmLowerEncoderIDA, kArmLowerEncoderIDB);
    ArmUpperAngleOffset=-90;
    ArmUpperAngleOffset=0.0;
    
    // ArmUpperEncoder.setReverseDirection(true);
    // ArmUpperMotor.setInverted(true);
    // ArmLowerEncoder.setReverseDirection(true);
    // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

    ArmUpperEncoder.setDistancePerPulse(kArmUpperEncoderConversion);
    ArmLowerEncoder.setDistancePerPulse(kArmLowerEncoderConversion);
    ArmUpperAngleTarget=ArmUpperAngleOffset;
    ArmLowerAngleTarget=ArmLowerAngleOffset;
    ArmUpperEncoder.reset();
    ArmLowerEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ArmDrive();
    // ArmUpperMotor.set(ControlMode.PercentOutput,1);
    // ArmLowerMotor.set(ControlMode.PercentOutput, 1);
    updateWidgets();
  }
  public double[] getArmAngles(){
    double[] result = {ArmUpperEncoder.getDistance()+ArmUpperAngleOffset,ArmLowerEncoder.getDistance()+ArmLowerAngleOffset};
    return result;
  }
  
  public void ArmDrive(){
    ArmUpperMotor.set(TalonSRXControlMode.PercentOutput,kDrive(ArmUpperAngleTarget-getArmAngles()[0]));
    ArmLowerMotor.set(TalonSRXControlMode.PercentOutput,kDrive(ArmLowerAngleTarget-getArmAngles()[1]));
  }
  public void ArmDrive(double shoulder,double lower){
    ArmUpperMotor.set(TalonSRXControlMode.PercentOutput,shoulder/100);
    ArmLowerMotor.set(TalonSRXControlMode.PercentOutput,lower/100);
  }

  public double kDrive(double pv){
    if(pv<0)return -.1;
    if(pv>0)return .1;
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
          ArmUpperAngleTarget=Math.toDegrees(phi1-Math.atan(targetY/targetX));
          ArmLowerAngleTarget=Math.toDegrees(phi2-Math.PI);
        } else{
          ArmUpperAngleTarget=-Math.toDegrees(phi1-Math.atan(targetY/targetX));
          ArmLowerAngleTarget=-Math.toDegrees(phi2-Math.PI);
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

  
  public void updateWidgets(){
    armUpperAngleWidget.setDouble(getArmAngles()[0]);
    armLowerAngleWidget.setDouble(getArmAngles()[1]);
    armUpperTargetWidget.setDouble(ArmUpperAngleTarget);
    armLowerTargetWidget.setDouble(ArmLowerAngleTarget);
  }
}
