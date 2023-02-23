// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AFFShufflable;

import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.utils.mapping.*;

public class Arm extends SubsystemBase {
    public AFFShufflable ShoulderFeedForward;
    public AFFShufflable ElbowFeedForward;
    public double ShoulderTarget;
    public double ElbowTarget;

    private CANSparkMax ShoulderMotor; // change to cansparkmax
    private DutyCycleEncoder ShoulderEncoder; // change to absolute encoder

    private DigitalInput ShoulderLimitSwitch;
    public boolean shoulderUpperLimit, shoulderLowerLimit;

    private CANSparkMax ElbowMotor; // change to cansparkmax
    private DutyCycleEncoder ElbowEncoder; // change to absolute encoder
    // change angle offsets and arm segment lengths in constants

    public double[][] targetSequence;
    public int sequenceIndex;

    /** Creates a new Arm. */
    public Arm() {
        // ShoulderMotor = new TalonSRX(kShoulderMotorID);
        // ShoulderEncoder = new Encoder(kShoulderEncoderIDA, kShoulderEncoderIDB);// ,
        // false, EncodingType.k4X)
        // ElbowMotor = new TalonSRX(kElbowMotorID);
        // ElbowEncoder = new Encoder(kElbowEncoderIDA, kElbowEncoderIDB);// , false,
        // EncodingType.k4X
        ShoulderMotor = new CANSparkMax(kShoulderMotorID, MotorType.kBrushless);
        ElbowMotor = new CANSparkMax(kElbowMotorID, MotorType.kBrushless);
        ShoulderEncoder = new DutyCycleEncoder(kShoulderEncoderIDA);// update encoder ID's
        ElbowEncoder = new DutyCycleEncoder(kElbowEncoderIDA);
        ShoulderLimitSwitch = new DigitalInput(kShoulderLimitSwitch);
        // ArmUpperEncoder.setReverseDirection(true);
        // ArmUpperMotor.setInverted(true);
        // ArmLowerEncoder.setReverseDirection(true);
        // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

        ShoulderEncoder.setDistancePerRotation(kShoulderEncoderConversion);
        ElbowEncoder.setDistancePerRotation(kElbowEncoderConversion);
        ShoulderTarget = kShoulderOffset;
        ElbowTarget = kElbowOffset;
        ShoulderEncoder.reset();
        ElbowEncoder.reset();
        ShoulderMotor.setIdleMode(IdleMode.kBrake);
        ShoulderMotor.setIdleMode(IdleMode.kBrake);
        // ShoulderMotor.setInverted(true);
        // ElbowMotor.setInverted(true);

        ShoulderFeedForward = new AFFShufflable(0, 0, 0, 0, 0, "ShoulderPID");
        ElbowFeedForward = new AFFShufflable(0, 0, 0, 0, 0, "ElbowPID");

        targetSequence = null;
        sequenceIndex = 0;
        updateShufflables();
    }

    @Override
    public void periodic() {
        limitLogic();
        // This method will be called once per scheduler run
        // the arm ALWAYS tries to meet its target angles
        executeSequence();
        // add a filter of target angles
        ArmDrive();
        updateWidgets();
        updateShufflables();
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
                ShoulderEncoder.getDistance() + kShoulderOffset,
                ElbowEncoder.getDistance() + kElbowOffset };// update to utilize absolute encoders
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
     *         relative to origin, 1dimension double[] is xy coordinates;
     *         example - result[1][0] is the elbow x coordinate
     * @param spatialArmAngles could be used for either current arm spatial angles
     *                         or target angles in global space
     */
    public double[][] forwardKinematics(double[] spatialArmAngles, double[] armLengths) {
        double[][] result = new double[spatialArmAngles.length][2];
        for (int i = 0; i < spatialArmAngles.length; i++) {
            result[i][0] = armLengths[i] * Math.cos(Math.toRadians(spatialArmAngles[i]));
            result[i][1] = armLengths[i] * Math.sin(Math.toRadians(spatialArmAngles[i]));
            if (i > 0) {
                result[i][0] = result[i][0] + result[i - 1][0];
                result[i][1] = result[i][1] + result[i - 1][1];
            }
        }
        return result;

    }

    /**
     * drive motors to the target angles using PIDF; use subsystem ElbowTarget and
     * ShoulderTarget which are local targets
     */
    public void ArmDrive() {
        ElbowMotor.set(
                clamp(
                        safeZoneDrive(
                                ElbowFeedForward.calc(ElbowTarget - getLocalArmAngles()[1],
                                        (getLocalArmAngles()[0] + getLocalArmAngles()[1])),
                                getLocalArmAngles()[1], kElbowSafezone),
                        -1.0, 1.0));
        ShoulderMotor.set(
                clamp(
                        safeZoneDrive(
                                ShoulderFeedForward.calc(ShoulderTarget - getLocalArmAngles()[0],
                                        (getLocalArmAngles()[0]), 0 * ElbowFeedForward.getLoad()),
                                getLocalArmAngles()[0], kShoulderSafezone),
                        -1.0, 1.0));
    }

    /** cycle through current sequence of angle targets */
    public void executeSequence() {
        if (targetSequence!=null && sequenceIndex >= targetSequence.length) {
            targetSequence = null;
        }
        if (targetSequence == null) {
            sequenceIndex = 0;
        } else {
            if (atTargets()) {
                sequenceIndex++;
            }
            ShoulderTarget = targetSequence[sequenceIndex][0];
            ElbowTarget = targetSequence[sequenceIndex][1];
        }
    }

    // old turret tyrapXX logic
    /** put in periodic, run shoulder limit logic */
    public void limitLogic() {
        if (!getShoulderLimitSwitch()) {
            shoulderUpperLimit = false;
            shoulderLowerLimit = false;
        } else if (ShoulderMotor.getAppliedOutput() > 0 && getShoulderLimitSwitch() && // check if it is
                                                                                       // getAppliedOutput or
                                                                                       // getOutputCurrent
                shoulderLowerLimit == false) {
            shoulderUpperLimit = true;
        } else if (ShoulderMotor.getAppliedOutput() < 0 && getShoulderLimitSwitch() &&
                shoulderUpperLimit == false) {
            shoulderLowerLimit = true;
        }
    }

    /**
     * 
     * @param pv what motor output will be, percent mode
     * @return proccessed pv based off of limit switches
     */
    public double clampShoulderByLimits(double pv) {
        double out = pv;
        if (shoulderUpperLimit) {
            out = clamp(out, -1, 0);
        }
        if (shoulderLowerLimit) {
            out = clamp(out, 0, 1);
        }
        return out;
    }

    public boolean getShoulderLimitSwitch() {
        return ShoulderLimitSwitch.get();
    }

    /**
     * 
     * @return if both arm angles are at target values
     */
    public boolean atTargets() {
        return Math.abs(getLocalArmAngles()[0] - ShoulderTarget) < armThreshold.get()
                && Math.abs(getLocalArmAngles()[1] - ElbowTarget) < armThreshold.get();
    }

    /**
     * setter method for arm angle target sequence, each array of double[] within
     * double[][] is a pair of arm angle targets 0=shoulder 1=elbow
     */
    public void setSequence(double[][] TargetSequence) {
        targetSequence = TargetSequence;
    }

    /**
     * @deprecated
     *             alternative to arm drive, but arm P is not proportional and only
     *             constant
     */
    public void ArmBangBang() {
        ElbowMotor.set(safeZoneDrive(
                bangbangdrive(ElbowTarget - getLocalArmAngles()[1], kElbowMaxPercent), getLocalArmAngles()[1],
                kElbowSafezone));
        ShoulderMotor.set(
                safeZoneDrive(bangbangdrive(ShoulderTarget - getLocalArmAngles()[0], kShoulderMaxPercent),
                        getLocalArmAngles()[0],
                        kShoulderSafezone));
    }

    /**
     * @deprecated
     *             a method to feed in target values instead of setting in
     *             subsystem, delete other armdrives and arm drive alternatives
     * @param shoulder is pv of shoulder
     * @param elbow    is pv of elbow
     */
    public void JoystickDriveRawArm(double shoulder, double elbow) {
        ShoulderMotor.set(
                safeZoneDrive(bangbangdrive(shoulder, kShoulderMaxPercent), getLocalArmAngles()[0], kShoulderSafezone));
        ElbowMotor.set(
                safeZoneDrive(bangbangdrive(elbow, kElbowMaxPercent), getLocalArmAngles()[1], kElbowSafezone));
    }

    /**
     * input PID output, @return pid output so that current motor angle complies to
     * safe range of angles
     */
    public double safeZoneDrive(double pv, double motorAngle, double[] safeZone) {
        if (motorAngle >= safeZone[1] && pv > 0)
            return 0;
        if (motorAngle <= safeZone[0] && pv < 0)
            return 0;
        return pv;
    }

    /**
     * @deprecated, use mapping utils clamp
     * 
     * @return given original percent output, cap @param output to maximum magnitude
     *         of 1
     */
    public double capPercent(double output) {
        if (Math.abs(output) > 1) {
            return 1 * Math.signum(output);
        }
        return output;
    }

    /**
     * @deprecated
     *             like PID but worse, given pv use exactly maximum motor output
     *             percent
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
     * @param targetX  hand target x coordinate
     * @param targetY  hand target y coordinate
     * @param stowable stowable configuration allows arm to fold neatly in, not
     *                 stowable potentially allows for better pickup of game pieces
     * 
     */
    public void ArmIKSet(double targetX, double targetY, boolean stowable) {// where x and y are relative to shoulder
                                                                            // position //stow means it will stow
                                                                            // nicely so by default TRUE
        double r = Math.sqrt(squareOf(targetY) + squareOf(targetX));
        if (r < kElbowLength + kShoulderLength && r > Math.abs(kElbowLength - kShoulderLength)) {// check for
                                                                                                 // geometrically
                                                                                                 // possible
                                                                                                 // triangle
            double phi1 = Math.acos((squareOf(kShoulderLength) + squareOf(kElbowLength) - squareOf(r))
                    / (2 * kShoulderLength * kElbowLength));
            double phi2 = Math.asin(kElbowLength * Math.sin(phi1) / r);
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
     * alternative to armIKdrive where stowable is true by default
     * set target angles based off of spatial coordinates from the shoulder (XY
     * coordinates)
     * 
     * @param targetX
     * @param targetY
     */
    public void ArmIKSet(double targetX, double targetY) {
        ArmIKSet(targetX, targetY, true);
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
        shoulderRawWidget.setDouble(ShoulderEncoder.getAbsolutePosition());
        elbowRawWidget.setDouble(ElbowEncoder.getAbsolutePosition());
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
        if (armThreshold.detectChanges())
            armThreshold.subscribeAndSet();
    }

}
