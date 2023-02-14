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
    ShoulderMotor = new TalonSRX(kShoulderMotorID);
    ShoulderEncoder = new Encoder(kShoulderEncoderIDA, kShoulderEncoderIDB);// , false, EncodingType.k4X)
    ElbowMotor = new TalonSRX(kElbowMotorID);
    ElbowEncoder = new Encoder(kElbowEncoderIDA, kElbowEncoderIDB);// , false, EncodingType.k4X
    ShoulderAngleOffset = kShoulderOffset;
    ElbowAngleOffset = kElbowOffset;

    // ArmUpperEncoder.setReverseDirection(true);
    // ArmUpperMotor.setInverted(true);
    // ArmLowerEncoder.setReverseDirection(true);
    // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

    ShoulderEncoder.setDistancePerPulse(kArmUpperEncoderConversion);
    ElbowEncoder.setDistancePerPulse(kArmLowerEncoderConversion);
    ShoulderTarget = ShoulderAngleOffset;
    ElbowTarget = ElbowAngleOffset;
    ShoulderEncoder.reset();
    ElbowEncoder.reset();
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    ShoulderMotor.setInverted(true);
    ElbowMotor.setInverted(true);

    ShoulderFeedForward = new AFFShufflable(0, 0, 0, 0, 0, "ShoulderPID");
    ShoulderFeedForward.shuffleUpdatePID();
    ElbowFeedForward = new AFFShufflable(0, 0, 0, 0, 0, "ElbowPID");
    ElbowFeedForward.shuffleUpdatePID();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // right now; this will always try to match current arm angle to target angle
    ArmDrive();
    updateWidgets();
    updatePIDs();
  }
  /**return current arm angles as a double array length 2
   * angle[0] is shoulder angle
   * angle[1] is elbow angle
   */
  public double[] getArmAngles() {
    double[] result = { ShoulderEncoder.getDistance() + ShoulderAngleOffset,
        ElbowEncoder.getDistance() + ElbowAngleOffset };
    return result;
  }

  // drive to target value always
  /**clean PIDFF with safeties; method goes into teleop and always drives motors to target */
  public void ArmDrive() {
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,
        capPercent(safeZoneDrive(
            ElbowFeedForward.calc(ElbowTarget - getArmAngles()[1], (getArmAngles()[0] + getArmAngles()[1])),
            getArmAngles()[1], kElbowSafezone)));
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,
        capPercent(safeZoneDrive(ShoulderFeedForward.calc(ShoulderTarget - getArmAngles()[0], (getArmAngles()[0]),
            0 * ElbowFeedForward.getLoad()), getArmAngles()[0], kShoulderSafezone)));
  }

  // bang-bang to target value always
  /**alternative ArmDrive method, use constant percentages to drive motors to meet target angles
   * like very bad PID
   */
  public void ArmBangBang() {
    ElbowMotor.set(TalonSRXControlMode.PercentOutput, safeZoneDrive(
        bangbangdrive(ElbowTarget - getArmAngles()[1], kElbowMaxPercent), getArmAngles()[1], kElbowSafezone));
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput, safeZoneDrive(
        bangbangdrive(ShoulderTarget - getArmAngles()[0], kShoulderMaxPercent), getArmAngles()[0], kShoulderSafezone));
  }
  /**alternative ArmDrive method
   * delete all armdrive type methods and use this method in the manipuloatir/teleop command
   * diagnostic tool
   */
  public void JoystickDriveRawArm(double shoulder, double elbow) {
    ShoulderMotor.set(TalonSRXControlMode.PercentOutput,
        safeZoneDrive(bangbangdrive(shoulder, kShoulderMaxPercent), getArmAngles()[0], kShoulderSafezone));
    ElbowMotor.set(TalonSRXControlMode.PercentOutput,
        safeZoneDrive(bangbangdrive(elbow, kElbowMaxPercent), getArmAngles()[1], kElbowSafezone));
  }

  // filter PID and motor drive values to be within safe zone (PID goes in, safe
  // number goes out)
  /**make sure we are not going to drive the motor out of the safe zone of angles
   * @param pv current calculation from PID; what the motor percentage will become
   * @param motorAngle the current motor angle
   * @param safeZone the two-double array of safe angles; [0] is lower limit [1] is upper limit
   */
  public double safeZoneDrive(double pv, double motorAngle, double[] safeZone) {
    if (motorAngle >= safeZone[1] && pv > 0)
      return 0;
    if (motorAngle <= safeZone[0] && pv < 0)
      return 0;
    return pv;
  }
  /**make sure percent output is capped to one hundred percent 
   * @param pv current calculation from PID; what the motor percentage will become
  */
  public double capPercent(double output) {
    if (Math.abs(output) > 1) {
      return 1 * Math.signum(output);
    }
    return output;
  }

  // like PID but just constant drive
  /**for one motor only, set the percentage power to the output of this method
   * do not need this method
   */
  public double bangbangdrive(double pv, double motorMaxPercent) {
    if (Math.abs(pv) > kArmMotorDeadband)
      return motorMaxPercent * Math.signum(pv);
    return 0;
  }

  // prerequisites: view the robot so that the arm extends to the right
  // when all arm angles are zeroed, the arm sticks straight out to the right
  // positive theta is counter clockwise, negative theta is clockwise
  /**
   * 
   * set target angles based off of spatial coordinates from the shoulder (XY
   * coordinates)
   * inverse kinematics
   * 
   * @param targetX
   * @param targetY
   * @param stowable stowable configuration allows arm to fold neatly in, not
   *                 stowable potentially allows for better pickup of game pieces
   * 
   */
  public void ArmIKDrive(double targetX, double targetY, boolean stowable) {// where x and y are relative to shoulder
                                                                            // position //stow means it will stow nicely
                                                                            // so by default TRUE
    double r = Math.sqrt(squareOf(targetY) + squareOf(targetX));
    if (r < kArmLowerLength + kArmUpperLength && r > Math.abs(kArmLowerLength - kArmUpperLength)) {// check for
                                                                                                   // geometrically
                                                                                                   // possible triangle
      double phi1 = Math.acos((squareOf(kArmUpperLength) + squareOf(kArmLowerLength) - squareOf(r))
          / (2 * kArmUpperLength * kArmLowerLength));
      double phi2 = Math.asin(kArmLowerLength * Math.sin(phi1) / r);
      // double phi3=Math.PI-(phi1+phi2);
      if (!stowable) {
        ShoulderTarget = Math.toDegrees(phi1 - Math.atan(targetY / targetX));
        ElbowTarget = Math.toDegrees(phi2 - Math.PI);
      } else {
        ShoulderTarget = -Math.toDegrees(phi1 - Math.atan(targetY / targetX));
        ElbowTarget = -Math.toDegrees(phi2 - Math.PI);
      }
    }
  }

  // the default mode
  /**
   * set target angles based off of spatial coordinates from the shoulder (XY
   * coordinates)
   * inverse kinematics
   * 
   * @param targetX
   * @param targetY
   *                 by default this is stowable configuration, third paramater
   * @param stowable decides configuration
   * 
   */
  public void ArmIKDrive(double targetX, double targetY) {
    ArmIKDrive(targetX, targetY, true);
  }

  private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private GenericPublisher shoulderAngleWidget = mainTab.add("ShoulderAngle", 0.0).withPosition(0, 3).withSize(1, 1)
      .getEntry();
  private GenericPublisher elbowAngleWidget = mainTab.add("elbowAngle", 0.0).withPosition(1, 3).withSize(1, 1)
      .getEntry();
  private GenericPublisher shoulderTargetWidget = mainTab.add("shouldertarget", 0.0).withPosition(2, 3).withSize(1, 1)
      .getEntry();
  private GenericPublisher elbowTargetWidget = mainTab.add("elbowTarget", 0.0).withPosition(3, 3).withSize(1, 1)
      .getEntry();
  private GenericPublisher shoulderRawWidget = mainTab.add("shoulderRaw", 0.0).withPosition(4, 3).withSize(1, 1)
      .getEntry();
  private GenericPublisher elbowRawWidget = mainTab.add("elbowRaw", 0.0).withPosition(5, 3).withSize(1, 1).getEntry();
  private GenericPublisher shoulderDrivePercentageWidget = mainTab.add("ShoulderDrive", 0.0).withPosition(6, 3)
      .withSize(1, 1).getEntry();
  private GenericPublisher elbowDrivePercentWidget = mainTab.add("elbowDrive", 0.0).withPosition(7, 3).withSize(1, 1)
      .getEntry();

  public void updateWidgets() {
    shoulderAngleWidget.setDouble(getArmAngles()[0]);
    elbowAngleWidget.setDouble(getArmAngles()[1]);
    shoulderTargetWidget.setDouble(ShoulderTarget);
    elbowTargetWidget.setDouble(ElbowTarget);
    shoulderRawWidget.setDouble(ShoulderEncoder.getRaw());
    elbowRawWidget.setDouble(ElbowEncoder.getRaw());
    elbowDrivePercentWidget.setDouble(
        safeZoneDrive(ElbowFeedForward.calc(ElbowTarget - getArmAngles()[1], getArmAngles()[0] + getArmAngles()[1]),
            getArmAngles()[1], kElbowSafezone));
    shoulderDrivePercentageWidget.setDouble(safeZoneDrive(
        ShoulderFeedForward.calc(ShoulderTarget - getArmAngles()[0], getArmAngles()[0], 0 * ElbowFeedForward.getLoad()),
        getArmAngles()[0], kShoulderSafezone));
  }

  public void updatePIDs() {
    if (ElbowFeedForward.detectChange())
      ElbowFeedForward.shuffleUpdatePID();
    if (ShoulderFeedForward.detectChange())
      ShoulderFeedForward.shuffleUpdatePID();
  }

}
