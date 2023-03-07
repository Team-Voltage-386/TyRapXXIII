// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AFFShufflable;
import frc.robot.utils.PersistentShufflableDouble;
import static frc.robot.utils.TrajectoryMaker.*;
import frc.robot.utils.ArmKeyframe.armKeyFrameStates;
import frc.robot.utils.ArmKeyframe;
import frc.robot.utils.Flags;

import static frc.robot.utils.Flags.*;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmConstants.ArmSequences.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.utils.mapping.*;

import java.util.Map;

public class Arm extends SubsystemBase {

    public AFFShufflable ShoulderFeedForward;
    public AFFShufflable ElbowFeedForward;
    public double ShoulderTarget;
    public double ElbowTarget;

    private CANSparkMax ShoulderMotor; // change to cansparkmax
    private DutyCycleEncoder ShoulderEncoder; // change to absolute encoder

    private DigitalInput ShoulderLimitSwitch; // LIMIT READING TRUE MEANS SWTICH NOT HIT
    public boolean shoulderUpperLimit, shoulderLowerLimit;

    private CANSparkMax ElbowMotor; // change to cansparkmax
    private DutyCycleEncoder ElbowEncoder; // change to absolute encoder
    // change angle offsets and arm segment lengths in constants
    public ArmKeyframe[] keyFrameSequence;
    public double[][] targetSequence;
    public ArmKeyframe lastKeyframe, nextKeyframe;
    public double[][] fkCoords;
    public int keyFrameIndex, sequenceIndex;
    public boolean runningKeyframesAndSequences;

    // public PersistentShufflableDouble PSDInitialShoulderTarget,
    // PSDInitialElbowTarget;

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
        ShoulderEncoder = new DutyCycleEncoder(kShoulderEncoderPin);// update encoder ID's
        ElbowEncoder = new DutyCycleEncoder(kElbowEncoderPin);
        ShoulderLimitSwitch = new DigitalInput(kShoulderLimitSwitch);
        // ArmUpperEncoder.setReverseDirection(true);
        // ArmUpperMotor.setInverted(true);
        // ArmLowerEncoder.setReverseDirection(true);
        // ArmLowerMotor.setInverted(InvertType.InvertMotorOutput);

        ShoulderEncoder.setDistancePerRotation(kShoulderEncoderConversion);
        ElbowEncoder.setDistancePerRotation(kElbowEncoderConversion);
        // PSDInitialShoulderTarget = new PersistentShufflableDouble(0,
        // "shoulderTargPSD", "Arm");
        // PSDInitialElbowTarget = new PersistentShufflableDouble(0, "ElbowTargPSD",
        // "Arm");
        ShoulderEncoder.reset();
        ElbowEncoder.reset();

        ShoulderFeedForward = new AFFShufflable(.0001, 0, 0, 0, 0, "ShoulderPIDF", "ArmFF");
        ElbowFeedForward = new AFFShufflable(.0001, 0, 0, 0, 0, "ElbowPIDF", "ArmFF");

        sequenceIndex = 0;

        updateShufflables();

        ShoulderTarget = kInitialShoulderTarget;
        ElbowTarget = kInitialElbowTarget;
        targetSequence = new double[][] { { kInitialShoulderTarget, kInitialElbowTarget } };
        lastKeyframe = new ArmKeyframe(new double[] { ShoulderTarget, ElbowTarget }, armKeyFrameStates.stowed, 0);
        nextKeyframe = new ArmKeyframe(new double[] { ShoulderTarget, ElbowTarget }, armKeyFrameStates.stowed, 0);
        keyFrameSequence = new ArmKeyframe[] { lastKeyframe };
        // keyFrameSequence = onlyIntermediary1(akfStowed);
        runningKeyframesAndSequences = false;
    }

    @Override
    // This method will be called once per scheduler run

    public void periodic() {
        limitLogic();
        setFlags();
        fkCoords = forwardKinematics(getSpatialArmAngles(getLocalArmAngles()), kArmLengths);
        if (runningKeyframesAndSequences) {
            executeKeyframesAndSequences();
        }

        // add a filter of target angles here
        ArmDrive();// the only line that will drive the arm motors is this one
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
                armAnglesIncludeDepression(ShoulderEncoder.getAbsolutePosition() * 360
                        - kShoulderEncOffset),
                armAnglesIncludeDepression(ElbowEncoder.getAbsolutePosition() * 360 - kElbowEncOffset) };
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
        double elbowErr = ElbowTarget - getLocalArmAngles()[1];
        double shouldErr = ShoulderTarget - getLocalArmAngles()[0];
        // if (Math.abs(elbowErr) > 30)
        //     ElbowFeedForward.integralAcc = 0;
        if (Math.abs(shouldErr) > 15)
            ShoulderFeedForward.integralAcc = 0;
        switch (nextKeyframe.keyFrameState) {
            case stowed:
                if (!runningKeyframesAndSequences) {
                    ElbowMotor.setVoltage(KStowPressVelocity);
                    ElbowFeedForward.integralAcc = 0;
                } else {
                    ElbowMotor.setVoltage(
                            clamp(
                                    safeZoneDrive(
                                            ElbowFeedForward.calc(elbowErr,
                                                    (getSpatialArmAngles(getLocalArmAngles())[1])),
                                            getLocalArmAngles()[1], kElbowSafezone),
                                    -PSDElbowMaxVoltage.get(), PSDElbowMaxVoltage.get()));
                }
                break;
            default:
                ElbowMotor.setVoltage(
                        clamp(
                                safeZoneDrive(
                                        ElbowFeedForward.calc(elbowErr,
                                                (getSpatialArmAngles(getLocalArmAngles())[1])),
                                        getLocalArmAngles()[1], kElbowSafezone),
                                -PSDElbowMaxVoltage.get(), PSDElbowMaxVoltage.get()));
                break;
        }

        ShoulderMotor.setVoltage(
                clampShoulderByLimits(clamp(
                        safeZoneDrive(
                                ShoulderFeedForward.calc(
                                        shouldErr,
                                        (getLocalArmAngles()[0]), 0 * ElbowFeedForward.getLoad()), // 0 is a constant
                                getLocalArmAngles()[0], kShoulderSafezone),
                        -PSDShoulderMaxVoltage.get(), PSDShoulderMaxVoltage.get())));
    }

    public double[][] zipperAngles(double[] shoulderAngles, double[] elbowAngles) {
        double[][] result = new double[shoulderAngles.length][2];
        for (int i = 0; i < shoulderAngles.length; i++) {
            result[i][0] = shoulderAngles[i];
            result[i][1] = elbowAngles[i];
        }
        return result;
    }

    public double[] unzipAngles(double[][] targetSequenceZipped, int index) {
        double[] result = new double[targetSequenceZipped.length];
        for (int i = 0; i < targetSequenceZipped.length; i++) {
            result[i] = targetSequenceZipped[i][index];
        }
        return result;
    }

    public void executeKeyframesAndSequences() {
        // at the very beginning
        System.out.println(keyFrameIndex + " " + sequenceIndex);
        if (keyFrameIndex == 0 && sequenceIndex == 0) {
            lastKeyframe = new ArmKeyframe(getLocalArmAngles(), armKeyFrameStates.intermediary, 3);
            // lastKeyframe.keyFrameState,
            // lastKeyframe.substepsToHere);
            nextKeyframe = keyFrameSequence[0];
            targetSequence = zipperAngles(
                    generateTrajectory(
                            lastKeyframe.getKeyFrameAngles()[0], nextKeyframe.getKeyFrameAngles()[0],
                            nextKeyframe.substepsToHere),
                    generateTrajectory(
                            lastKeyframe.getKeyFrameAngles()[1], nextKeyframe.getKeyFrameAngles()[1],
                            nextKeyframe.substepsToHere));
            sequenceIndex++;
            // reset i term
        }
        // at a keyframe, not the end or middle
        else if (keyFrameIndex != 0 && keyFrameIndex < keyFrameSequence.length && sequenceIndex == 0) {
            lastKeyframe = nextKeyframe;
            nextKeyframe = keyFrameSequence[keyFrameIndex];
            targetSequence = zipperAngles(
                    generateTrajectory(
                            lastKeyframe.getKeyFrameAngles()[0], nextKeyframe.getKeyFrameAngles()[0],
                            nextKeyframe.substepsToHere),
                    generateTrajectory(
                            lastKeyframe.getKeyFrameAngles()[1], nextKeyframe.getKeyFrameAngles()[1],
                            nextKeyframe.substepsToHere));
            sequenceIndex++;
            // reset i term
        }
        // at end
        else if (keyFrameIndex >= keyFrameSequence.length) {
            lastKeyframe = nextKeyframe;
            // break, it is done
            runningKeyframesAndSequences = false;
        }
        // between keyframes
        else if (sequenceIndex != 0 && sequenceIndex < targetSequence.length) {
            //
            ShoulderTarget = targetSequence[sequenceIndex][0];
            ElbowTarget = targetSequence[sequenceIndex][1];
            if (atTargets()) {
                sequenceIndex++;
            }
            if (sequenceIndex >= targetSequence.length) {
                sequenceIndex = 0;
                keyFrameIndex++;
            }
        }
    }

    /** call to set global flags based off of arm state */
    public void setFlags() {
        switch (lastKeyframe.keyFrameState) {
            case stowed:
                handCanRotate = false;
                break;
            case score:
                handCanRotate = true;
                break;
            case pickup:
                handCanRotate = true;
                break;
            default:
                handCanRotate = true;
                break;
        }
        armIsAtTarget = !runningKeyframesAndSequences && atTargets();
    }

    // old turret tyrapXX logic
    /** put in periodic, run shoulder limit logic */
    public void limitLogic() {
        if (!getShoulderLimitSwitch()) {
            shoulderUpperLimit = false;
            shoulderLowerLimit = false;
            // TEMPORARY LOGIC, will be problematic if shouldertarget==getlocalarmangles[0]
        } else if (getLocalArmAngles()[0] > kShoulderMiddleAngleThreshold && getShoulderLimitSwitch() &&
                shoulderLowerLimit == false) {
            shoulderUpperLimit = true;
        } else if (getLocalArmAngles()[0] < kShoulderMiddleAngleThreshold && getShoulderLimitSwitch() &&
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
            out = clamp(out, -PSDShoulderMaxVoltage.get(), 0);
        }
        if (shoulderLowerLimit) {
            out = clamp(out, 0, PSDShoulderMaxVoltage.get());
        }
        return out;
    }

    /**
     * 
     * @return true IF HITTING limit switch, false if not at limit switch
     */
    public boolean getShoulderLimitSwitch() {
        return !ShoulderLimitSwitch.get();
    }

    /**
     * 
     * @return if both arm angles are at target values
     */
    public boolean atTargets() {
        return Math.abs(getLocalArmAngles()[0] - ShoulderTarget) < kArmTolerance
                && Math.abs(getLocalArmAngles()[1] - ElbowTarget) < kArmTolerance;
    }

    /**
     * setter method for arm angle target sequence, each array of double[] within
     * double[][] is a pair of arm angle targets 0=shoulder 1=elbow
     */
    public boolean setKeyFrameSequence(ArmKeyframe[] inputKeyFrames) {
        if (keyFrameSequence != inputKeyFrames && !runningKeyframesAndSequences) {
            keyFrameIndex = 0;
            sequenceIndex = 0;
            keyFrameSequence = inputKeyFrames;
            runningKeyframesAndSequences = true;
            return true;
        } return false;
    }

    // /**
    // * @deprecated
    // * alternative to arm drive, but arm P is not proportional and only
    // * constant
    // */
    // public void ArmBangBang() {
    // ElbowMotor.set(safeZoneDrive(
    // bangbangdrive(ElbowTarget - getLocalArmAngles()[1],
    // PSDElbowMaxPercentage.get()),
    // getLocalArmAngles()[1],
    // kElbowSafezone));
    // ShoulderMotor.set(
    // safeZoneDrive(bangbangdrive(ShoulderTarget - getLocalArmAngles()[0],
    // PSDShoulderMaxPercentage.get()),
    // getLocalArmAngles()[0],
    // kShoulderSafezone));
    // }

    // /**
    // * @deprecated
    // * a method to feed in target values instead of setting in
    // * subsystem, delete other armdrives and arm drive alternatives
    // * @param shoulder is pv of shoulder
    // * @param elbow is pv of elbow
    // */
    // public void JoystickDriveRawArm(double shoulder, double elbow) {
    // ShoulderMotor.set(
    // safeZoneDrive(bangbangdrive(shoulder, PSDShoulderMaxPercentage.get()),
    // getLocalArmAngles()[0],
    // kShoulderSafezone));
    // ElbowMotor.set(
    // safeZoneDrive(bangbangdrive(elbow, PSDElbowMaxPercentage.get()),
    // getLocalArmAngles()[1],
    // kElbowSafezone));
    // }

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

    // /**
    // * @deprecated, use mapping utils clamp
    // *
    // * @return given original percent output, cap @param output to maximum
    // magnitude
    // * of 1
    // */
    // public double capPercent(double output) {
    // if (Math.abs(output) > 1) {
    // return 1 * Math.signum(output);
    // }
    // return output;
    // }

    // /**
    // * @deprecated
    // * like PID but worse, given pv use exactly maximum motor output
    // * percent
    // */
    // public double bangbangdrive(double pv, double motorMaxPercent) {
    // if (Math.abs(pv) > kArmMotorDeadband)
    // return motorMaxPercent * Math.signum(pv);
    // return 0;
    // }

    // // prerequisites: view the robot so that the arm extends to the right
    // // when all arm angles are zeroed, the arm sticks straight out to the right
    // // positive theta is counter clockwise, negative theta is clockwise
    // /**
    // * @deprecated (by Juan)
    // * set target angles based off of spatial coordinates from the
    // * shoulder (XY
    // * coordinates)
    // * inverse kinematics
    // *
    // * @param targetX hand target x coordinate
    // * @param targetY hand target y coordinate
    // * @param stowable stowable configuration allows arm to fold neatly in, not
    // * stowable potentially allows for better pickup of game pieces
    // *
    // */
    // public void ArmIKSet(double targetX, double targetY, boolean stowable) {//
    // where x and y are relative to shoulder
    // // position //stow means it will stow
    // // nicely so by default TRUE
    // double r = Math.sqrt(squareOf(targetY) + squareOf(targetX));
    // if (r < kElbowLength + kShoulderLength && r > Math.abs(kElbowLength -
    // kShoulderLength)) {// check for
    // // geometrically
    // // possible
    // // triangle
    // double phi1 = Math.acos((squareOf(kShoulderLength) + squareOf(kElbowLength) -
    // squareOf(r))
    // / (2 * kShoulderLength * kElbowLength));
    // double phi2 = Math.asin(kElbowLength * Math.sin(phi1) / r);
    // // double phi3=Math.PI-(phi1+phi2);
    // if (!stowable) {
    // ShoulderTarget = Math.toDegrees(phi1 - Math.atan(targetY / targetX));
    // ElbowTarget = Math.toDegrees(phi2 - Math.PI);
    // } else {
    // ShoulderTarget = -Math.toDegrees(phi1 - Math.atan(targetY / targetX));
    // ElbowTarget = -Math.toDegrees(phi2 - Math.PI);
    // }
    // }
    // }

    // // the default mode
    // /**
    // * @deprecated (by Juan)
    // * alternative to armIKdrive where stowable is true by default
    // * set target angles based off of spatial coordinates from the
    // * shoulder (XY
    // * coordinates)
    // *
    // * @param targetX
    // * @param targetY
    // */
    // public void ArmIKSet(double targetX, double targetY) {
    // ArmIKSet(targetX, targetY, true);
    // }

    private ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    private GenericPublisher shoulderAngleWidget = armTab.add("ShoulderAngle", 0.0).withPosition(0, 0).withSize(1, 1)
            .getEntry();
    private GenericPublisher elbowAngleWidget = armTab.add("elbowAngle", 0.0).withPosition(1, 0).withSize(1, 1)
            .getEntry();
    private GenericPublisher shoulderTargetWidget = armTab.add("shouldertarget", 0.0).withPosition(0, 1).withSize(1, 1)
            .getEntry();
    private GenericPublisher elbowTargetWidget = armTab.add("elbowTarget", 0.0).withPosition(1, 1).withSize(1, 1)
            .getEntry();
    // raw angle widgets
    // private GenericPublisher shoulderRawWidget = armTab.add("shoulderRaw",
    // 0.0).withSize(1, 1)
    // .getEntry();
    // private GenericPublisher elbowRawWidget = armTab.add("elbowRaw",
    // 0.0).withSize(1, 1).getEntry();
    // motor percentages
    private GenericPublisher shoulderDrivePercentageWidget = armTab.add("ShoulderDrive", 0.0)
            .withPosition(0, 2).withSize(1, 1).getEntry();
    private GenericPublisher elbowDrivePercentWidget = armTab.add("elbowDrive", 0.0).withPosition(1, 2).withSize(1, 1)
            .getEntry();
    // private GenericPublisher shoulderDriveAmpsWidget = armTab.add("ShoulderAmps",
    // 0.0).getEntry();
    // private GenericPublisher elbowDriveAmpsWidget = armTab.add("ElbowAMps",
    // 0.0).getEntry();
    // private GenericPublisher shoulderTemperatureWidget =
    // armTab.add("ShoulderTemp", 0.0).getEntry();
    // private GenericPublisher elbowTemperatureWidget = armTab.add("ElbowTemp",
    // 0.0).getEntry();
    // these are shoulder limit switch widgets
    // private GenericPublisher shoulderLimitWidget = armTab.add("shoulderLimit",
    // false).withSize(1, 1)
    // .getEntry();
    private GenericPublisher shoulderUpperLimitWidget = armTab.add("upperLimit", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#009900"))
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();
    private GenericPublisher shoulderLowerLimitWidget = armTab.add("lowerLimit", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#009900"))
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();
    // // these are FK coordinate widgets
    // private GenericPublisher shoulderXWidget = armTab.add("elbowX",
    // 0.0).withPosition(5, 1).withSize(1, 1)
    // .getEntry();
    // private GenericPublisher shoulderYWidget = armTab.add("elbowY",
    // 0.0).withPosition(6, 1).withSize(1, 1)
    // .getEntry();
    // private GenericPublisher elbowXWidget = armTab.add("handX",
    // 0.0).withPosition(5, 2).withSize(1, 1)
    // .getEntry();
    // private GenericPublisher elbowYWidget = armTab.add("handY",
    // 0.0).withPosition(6, 2).withSize(1, 1).getEntry();
    private GenericPublisher lastKeyFrameWidget = armTab.add("lastKeyFrame", "").withPosition(3, 2).getEntry();
    private GenericPublisher nextKeyFrameWidget = armTab.add("nextKeyFrame", "").withPosition(3, 3).getEntry();
    private GenericPublisher shoulderTargetSequenceWidget = armTab.add("shoudertargets", new double[] {})
            .withPosition(4, 0).getEntry();
    private GenericPublisher elbowTargetSequenceWidget = armTab.add("elbowTargets", new double[] {})
            .withPosition(4, 1).getEntry();
    private GenericPublisher keyFrameIndexWidget = armTab.add("keyFrameIndex", 0).withPosition(4, 2).getEntry();
    private GenericPublisher targetIndexWidget = armTab.add("targetIndex", 0).withPosition(4, 3).getEntry();
    private GenericPublisher atTargetsWidget = armTab.add("atTargets", false).withPosition(5, 0).getEntry();
    private GenericPublisher runningKeyframesAndSequencesWidget = armTab.add("running", false).withPosition(4, 0)
            .getEntry();

    public GenericEntry ConeModeWidget = armTab.add("coneMode", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#9900FF")).withPosition(5, 1)
            .getEntry();
    public GenericEntry scoreHighWidget = armTab.add("scoreHigh", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "#FFFFFF", "Color when false", "#999999")).withPosition(5, 2)
            .getEntry();

    public void updateWidgets() {
        shoulderAngleWidget.setDouble(getLocalArmAngles()[0]);
        elbowAngleWidget.setDouble(getLocalArmAngles()[1]);
        shoulderTargetWidget.setDouble(ShoulderTarget);
        elbowTargetWidget.setDouble(ElbowTarget);
        // shoulderRawWidget.setDouble(ShoulderEncoder.getAbsolutePosition());
        // elbowRawWidget.setDouble(ElbowEncoder.getAbsolutePosition());
        elbowDrivePercentWidget.setDouble(ElbowMotor.getAppliedOutput());
        shoulderDrivePercentageWidget.setDouble(ShoulderMotor.getAppliedOutput());
        // elbowDriveAmpsWidget.setDouble(ElbowMotor.getOutputCurrent());
        // shoulderDriveAmpsWidget.setDouble(ShoulderMotor.getOutputCurrent());
        // elbowTemperatureWidget.setDouble(ElbowMotor.getMotorTemperature());
        // shoulderTemperatureWidget.setDouble(ShoulderMotor.getMotorTemperature());

        // shoulderLimitWidget.setBoolean(ShoulderLimitSwitch.get());
        shoulderUpperLimitWidget.setBoolean(shoulderUpperLimit);
        shoulderLowerLimitWidget.setBoolean(shoulderLowerLimit);
        // shoulderXWidget.setDouble(forwardKinematics(getSpatialArmAngles(getLocalArmAngles()),
        // kArmLengths)[0][0]);
        // shoulderYWidget.setDouble(forwardKinematics(getSpatialArmAngles(getLocalArmAngles()),
        // kArmLengths)[0][1]);
        // elbowXWidget.setDouble(forwardKinematics(getSpatialArmAngles(getLocalArmAngles()),
        // kArmLengths)[1][0]);
        // elbowYWidget.setDouble(forwardKinematics(getSpatialArmAngles(getLocalArmAngles()),
        // kArmLengths)[1][1]);
        lastKeyFrameWidget.setString(lastKeyframe.stateString());
        nextKeyFrameWidget.setString(nextKeyframe.stateString());
        keyFrameIndexWidget.setInteger(keyFrameIndex);
        targetIndexWidget.setInteger(sequenceIndex);
        atTargetsWidget.setBoolean(atTargets());
        if (targetSequence != null) {
            shoulderTargetSequenceWidget.setDoubleArray(unzipAngles(targetSequence, 0));
            elbowTargetSequenceWidget.setDoubleArray(unzipAngles(targetSequence, 1));
        }
        ConeModeWidget.setBoolean(ConeMode);
        runningKeyframesAndSequencesWidget.setBoolean(runningKeyframesAndSequences);

        scoreHighWidget.setBoolean(scoreHighTarget);
    }

    public void updateShufflables() {
        if (ElbowFeedForward.detectChange())
            ElbowFeedForward.shuffleUpdatePID();
        if (ShoulderFeedForward.detectChange())
            ShoulderFeedForward.shuffleUpdatePID();
        if (PSDArmTolerace.detectChanges())
            PSDArmTolerace.subscribeAndSet();
        if (PSDElbowOffset.detectChanges())
            PSDElbowOffset.subscribeAndSet();
        if (PSDShoulderOffset.detectChanges())
            PSDShoulderOffset.subscribeAndSet();
        if (PSDShoulderMaxVoltage.detectChanges())
            PSDShoulderMaxVoltage.subscribeAndSet();
        if (PSDElbowMaxVoltage.detectChanges())
            PSDElbowMaxVoltage.subscribeAndSet();
        if (PSDStowPressVelocity.detectChanges())
            PSDStowPressVelocity.subscribeAndSet();
    }

}
