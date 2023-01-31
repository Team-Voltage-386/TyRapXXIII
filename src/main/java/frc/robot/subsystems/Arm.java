// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Arm extends SubsystemBase {
  public double ArmUpperAngleTarget;
  public double ArmLowerAngleTarget;

  private TalonSRX ArmUpperMotor;
  private Encoder ArmUpperEncoder;

  private TalonSRX ArmLowerMotor;
  private Encoder ArmLowerEncoder;
  /** Creates a new Arm. */
  public Arm() {
    ArmUpperMotor=new TalonSRX(kArmUpperMotorID);
    ArmUpperEncoder=new Encoder(kArmUpperEncoderIDA, kArmUpperEncoderIDB);
    ArmLowerMotor=new TalonSRX(kArmLowerMotorID);
    ArmLowerEncoder = new Encoder(kArmLowerEncoderIDA, kArmLowerEncoderIDB);

    ArmUpperEncoder.reset();
    ArmLowerEncoder.reset();
    ArmUpperEncoder.setDistancePerPulse(kArmUpperEncoderConversion);
    ArmLowerEncoder.setDistancePerPulse(kArmLowerEncoderConversion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double[] getArmAngles(){
    double[] result = {(double)ArmUpperEncoder.getDistance(),(double)ArmLowerEncoder.getDistance()};
    return result;
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

  
}
