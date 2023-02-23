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
import frc.robot.utils.PersistentShufflableDouble;

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

    public double[][] targetSequence;
    public int sequenceIndex;

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
        ElbowFeedForward = new AFFShufflable(0, 0, 0, 0, 0, "ElbowPID");

        targetSequence = null;
        sequenceIndex = 0;
        updateShufflables();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // the arm ALWAYS tries to meet its target angles
        updateWidgets();
        updateShufflables();
        executeSequence();
        ArmDrive();

    }

    /**
     * 
     * @return double[] of current local arm angles in degrees, indeces 0=shoulder
     *         angle
     *         1=elbow angle, where arm angles are angles of elevation from local
     *         horizon (angle of depression is negative)
     */
    public double[] getLocalArmAngles() {
        double[] result = {
                ShoulderEncoder.getDistance() + ShoulderAngleOffset,
                ElbowEncoder.getDistance() + ElbowAngleOffset };// update to utilize absolute encoders
        return result;
    }

    /**
     * 
     * @return double[] of current spatial arm angles in degrees, indeces 0=shoulder
     *         angle 1=elbow angle, where arm angles are angles of elevation from
     *         global horizon (angle of depression is negative)
     */
    public double[] getSpatialArmAngles(double[] localArmAngles) {
        double[] result = new double[localArmAngles.length];
        for (int i = 0; i < localArmAngles.length; i++) {
            result[i] = localArmAngles[i];
            if (i > 0) {
                result[i] = result[i] + result[i - 1];
            }
        }
        return result;
    }

    /**
     * @return double[][] of coordinates of the endpoint of each arm segment
     *         relative to origin, 1d double[] is xy coordinates
     * @param spatialArmAngles could be used for either current arm spatial angles or target angles in global space
     */
    public double[][] forwardKinematics(double[] spatialArmAngles, double[] armLengths) {
        double[][] result = new double[spatialArmAngles.length][2];
        for (int i = 0; i < spatialArmAngles.length; i++) {
            result[i][0] = armLengths[i] * Math.cos(Math.toRadians(spatialArmAngles[i]));
            result[i][1] = armLengths[i] * Math.sin(Math.toRadians(spatialArmAngles[i]));
            if(i>0){
                result[i][0]=result[i][0]+result[i-1][0];
                result[i][1]=result[i][1]+result[i-1][1];
            }
        }
        return result;

    }

    /** drive motors to the target angles using PIDF */
    public void ArmDrive() {
        ElbowMotor.set(TalonSRXControlMode.PercentOutput,
                capPercent(safeZoneDrive(
                        ElbowFeedForward.calc(ElbowTarget - getLocalArmAngles()[1],
                                (getLocalArmAngles()[0] + getLocalArmAngles()[1])),
                        getLocalArmAngles()[1], kElbowSafezone)));
        ShoulderMotor
                .set(TalonSRXControlMode.PercentOutput,
                        capPercent(
                                safeZoneDrive(
                                        ShoulderFeedForward.calc(ShoulderTarget - getLocalArmAngles()[0],
                                                (getLocalArmAngles()[0]), 0 * ElbowFeedForward.getLoad()),
                                        getLocalArmAngles()[0], kShoulderSafezone)));
    }

    /** cycle through current sequence of angle targets */
    public void executeSequence() {
        if (targetSequence == null) {
            sequenceIndex = 0;
        } else {
            ShoulderTarget = targetSequence[sequenceIndex][0];
            ElbowTarget = targetSequence[sequenceIndex][1];
        }
        if (atTargets()) {
            sequenceIndex++;
        }
        if (sequenceIndex >= targetSequence.length) {
            targetSequence = null;
        }
    }

    /**
     * 
     * @return if both arm angles are at target values
     */
    public boolean atTargets() {
        return Math.abs(getLocalArmAngles()[0] - ShoulderTarget) < armDeadband.get()
                && Math.abs(getLocalArmAngles()[1] - ElbowTarget) < armDeadband.get();
    }

    /**
     * setter method for arm angle target sequence, each array of double[] within
     * double[][] is a pair of arm angle targets 0=shoulder 1=elbow
     */
    public void setSequence(double[][] TargetSequence) {
        targetSequence = TargetSequence;
    }

    /**@deprecated
     * alternative to arm drive, but arm P is not proportional and only constant
     */
    public void ArmBangBang() {
        ElbowMotor.set(TalonSRXControlMode.PercentOutput, safeZoneDrive(
                bangbangdrive(ElbowTarget - getLocalArmAngles()[1], kElbowMaxPercent), getLocalArmAngles()[1],
                kElbowSafezone));
        ShoulderMotor.set(TalonSRXControlMode.PercentOutput,
                safeZoneDrive(bangbangdrive(ShoulderTarget - getLocalArmAngles()[0], kShoulderMaxPercent),
                        getLocalArmAngles()[0],
                        kShoulderSafezone));
    }

    /**@deprecated
     * a method to feed in target values instead of setting in subsystem, delete other armdrives and arm drive alternatives
     * @param shoulder is pv of shoulder
     * @param elbow is pv of elbow
     */
    public void JoystickDriveRawArm(double shoulder, double elbow) {
        ShoulderMotor.set(TalonSRXControlMode.PercentOutput,
                safeZoneDrive(bangbangdrive(shoulder, kShoulderMaxPercent), getLocalArmAngles()[0], kShoulderSafezone));
        ElbowMotor.set(TalonSRXControlMode.PercentOutput,
                safeZoneDrive(bangbangdrive(elbow, kElbowMaxPercent), getLocalArmAngles()[1], kElbowSafezone));
    }

    
    /** input PID output, @return pid output so that current motor angle complies to safe range of angles */
    public double safeZoneDrive(double pv, double motorAngle, double[] safeZone) {
        if (motorAngle >= safeZone[1] && pv > 0)
            return 0;
        if (motorAngle <= safeZone[0] && pv < 0)
            return 0;
        return pv;
    }
    /**@return given original percent output, cap @param output to maximum magnitude of 1 */
    public double capPercent(double output) {
        if (Math.abs(output) > 1) {
            return 1 * Math.signum(output);
        }
        return output;
    }

    /**@deprecated
     * like PID but worse, given pv use exactly maximum motor output percent
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
     * set target angles based off of spatial coordinates from the shoulder (XY
     * coordinates)
     * inverse kinematics
     * 
     * @param targetX hand target x coordinate
     * @param targetY hand target y coordinate
     * @param stowable stowable configuration allows arm to fold neatly in, not
     *                 stowable potentially allows for better pickup of game pieces
     * 
     */
    public void ArmIKDrive(double targetX, double targetY, boolean stowable) {// where x and y are relative to shoulder
                                                                              // position //stow means it will stow
                                                                              // nicely so by default TRUE
        double r = Math.sqrt(squareOf(targetY) + squareOf(targetX));
        if (r < kArmLowerLength + kArmUpperLength && r > Math.abs(kArmLowerLength - kArmUpperLength)) {// check for
                                                                                                       // geometrically
                                                                                                       // possible
                                                                                                       // triangle
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
    /**alternative to armIKdrive where stowable is true by default
     * set target angles based off of spatial coordinates from the shoulder (XY
     * coordinates)
     * 
     * @param targetX
     * @param targetY
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
        shoulderAngleWidget.setDouble(getLocalArmAngles()[0]);
        elbowAngleWidget.setDouble(getLocalArmAngles()[1]);
        shoulderTargetWidget.setDouble(ShoulderTarget);
        elbowTargetWidget.setDouble(ElbowTarget);
        shoulderRawWidget.setDouble(ShoulderEncoder.getRaw());
        elbowRawWidget.setDouble(ElbowEncoder.getRaw());
        elbowDrivePercentWidget.setDouble(safeZoneDrive(
                ElbowFeedForward.calc(ElbowTarget - getLocalArmAngles()[1],
                        getLocalArmAngles()[0] + getLocalArmAngles()[1]),
                getLocalArmAngles()[1], kElbowSafezone));
        shoulderDrivePercentageWidget
                .setDouble(safeZoneDrive(
                        ShoulderFeedForward.calc(ShoulderTarget - getLocalArmAngles()[0], getLocalArmAngles()[0],
                                0 * ElbowFeedForward.getLoad()),
                        getLocalArmAngles()[0], kShoulderSafezone));
    }

    public void updateShufflables() {
        if (ElbowFeedForward.detectChange())
            ElbowFeedForward.shuffleUpdatePID();
        if (ShoulderFeedForward.detectChange())
            ShoulderFeedForward.shuffleUpdatePID();
        if (armDeadband.detectChanges())
            armDeadband.subscribeAndSet();
    }

}
