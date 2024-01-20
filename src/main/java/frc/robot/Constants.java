// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.ArmKeyframe;
import frc.robot.utils.PID;
// import frc.robot.utils.PIDShufflable;
import frc.robot.utils.PersistentShufflableDouble;
import frc.robot.utils.PersistentShufflableInteger;
import frc.robot.utils.ArmKeyframe.armKeyFrameStates;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        /** the indexes to address buttons on the controller */
        public static final class ControllerConstants {
                public static final double kDeadband = .1;
                public static final Joystick kDriver = new Joystick(0);
                public static final Joystick kManipulator = new Joystick(1);

                public static final int kLeftVertical = 1;
                public static final int kRightVertical = 5;
                public static final int kLeftHorizontal = 0;
                public static final int kRightHorizontal = 4;
                public static final int kLeftTrigger = 2;
                public static final int kRightTrigger = 3;

                public static final int kA = 1;
                public static final int kB = 2;
                public static final int kX = 3;
                public static final int kY = 4;
                public static final int kLeftBumper = 5;
                public static final int kRightBumper = 6;
                public static final int kLeftOptions = 7;
                public static final int kRightOptions = 8;
                public static final int kLeftJoystickPressed = 9;
                public static final int kRightJoystickPressed = 10;
        }

        public static final class AutoConstants {

                public static final double driveTolerance = 0.2;
                public static final double headingTolerance = 90;

                public static final double[] kAutoPositionPID = { .3, .075, .01 };
                public static final double[] kAutoHeadingPID = { 0.01, 0, 0 };
                public static final PID autoPositionX = new PID(kAutoPositionPID[0],
                                kAutoPositionPID[1],
                                kAutoPositionPID[2]);
                public static final PID autoPositionY = new PID(kAutoPositionPID[0],
                                kAutoPositionPID[1],
                                kAutoPositionPID[2]);
                public static final PID autoPositionH = new PID(kAutoHeadingPID[0],
                                kAutoHeadingPID[1],
                                kAutoHeadingPID[2]);
        }

        public static final class HandConstants {
                public static final double kMaxWristPower = 0.7;
                public static final double kWristSideTargetAngle = 10;
                public static final int kDoubleSolenoidModule = 0;
                public static final int kSolenoidForward = 0;
                public static final int kSolenoidReverse = 1;
                public static final PersistentShufflableDouble[] WristPIDPSDs = {
                                new PersistentShufflableDouble(0, "WristP", "Hand Tab"),
                                new PersistentShufflableDouble(0, "WristI", "Hand Tab"),
                                new PersistentShufflableDouble(0, "WristD", "Hand Tab"),
                };
                public static final Double[] kWristPID = { 0.0, 0.0, 0.0

                };
                public static final int kHandRotator = 2;
                public static final int kHandLimitSwitch = 3;

                public static final double kRotationSpeed = 0.50;

                public static double kConeIntakeSpeed = 0.20;
                public static double kCubeIntakeSpeed = 0.20;
                public static double kCubeStowSpeed = 0.10;
                public static final int kRightPickupID = 33;
                public static final int kLeftPickupID = 34;
        }

        /** Can IDs, PID values, ect. */
        public static final class DriveConstants {

                // public static final double kMaxRotSpeed = 180; // should be in degrees per
                // second
                // public static final double kMaxDriveSpeed = 4; // should be in meters per
                // second
                public static PersistentShufflableDouble PSDMaxRotSpeed = new PersistentShufflableDouble(180,
                                "maxRotationSpeed");
                public static PersistentShufflableDouble PSDMaxDriveSpeed = new PersistentShufflableDouble(4,
                                "maxDriveSpeed");
                public static final double kMaxRotSpeed = 120;
                public static final double kMaxDriveSpeed = 4; //SHOULD BE 4
                public static final double kSlowRotSpeed = 45;
                public static final double kSlowDriveSpeed = 1; //SHOULD BE 1
                public static final int kIMUid = 2;
                public static final double[] kSwerveSteerPID = { 0.006, 0.01, 0.0 }; // 0.01,0.0,0.001
                public static final double[] kSwerveDrivePID = { 0.3, 2, 1 }; // 0.35,2,0.01
                public static final double kSwerveDriveEncConv = 0.000745;

                // public static final SwerveModule LeftFront = new SwerveModule(14, 18,
                // kSwerveDriveEncConv, kSwerveSteerPID,
                // kSwerveDrivePID, 24, 0.36, -0.26, 84.1);
                // public static final SwerveModule RightFront = new SwerveModule(11, 15,
                // kSwerveDriveEncConv, kSwerveSteerPID,
                // kSwerveDrivePID, 21, 0.36, 0.26, 213.15);
                // public static final SwerveModule LeftRear = new SwerveModule(13, 17,
                // kSwerveDriveEncConv, kSwerveSteerPID,
                // kSwerveDrivePID, 23, -0.36, -0.26, 126.25);
                // public static final SwerveModule RightRear = new SwerveModule(12, 16,
                // kSwerveDriveEncConv, kSwerveSteerPID,
                // kSwerveDrivePID, 22, -0.36, 0.26, 292);
                public static final SwerveModule LeftFront = new SwerveModule(14, 18, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 24, 0.365125, -0.263525, 86.35, "LF", false);
                public static final SwerveModule RightFront = new SwerveModule(11, 15, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 21, 0.365125, 0.263525, 213.2, "RF", false);
                public static final SwerveModule LeftRear = new SwerveModule(13, 17, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 23, -0.365125, -0.263525, 235.45, "LR", false);// faulty encoder offset
                public static final SwerveModule RightRear = new SwerveModule(12, 16, kSwerveDriveEncConv,
                                kSwerveSteerPID,
                                kSwerveDrivePID, 22, -0.365125, 0.263525, 133.33, "RR", true);
        }

        public static final class SmoothingConstants {
                // public static final double kAccelerationSmoothFactor = .2;
                public static PersistentShufflableDouble kAccelerationSmoothFactor = new PersistentShufflableDouble(.2,
                                "driveAccelSmooth");
                // public static final double kRotationAccelerationSmoothFactor = .5;
                public static PersistentShufflableDouble kRotationAccelerationSmoothFactor = new PersistentShufflableDouble(
                                .5,
                                "rotateAccelSmooth");

        }

        public static final class ArmConstants {
                public static final PersistentShufflableDouble PSDShoulderMaxVoltage = new PersistentShufflableDouble(3,
                                "ShoulderMaxVoltage", "ArmFF");
                public static final PersistentShufflableDouble PSDElbowMaxVoltage = new PersistentShufflableDouble(3,
                                "ElbowMaxVoltage", "ArmFF");
                public static final PersistentShufflableDouble PSDStowPressVelocity = new PersistentShufflableDouble(
                                -.01,
                                "StowVoltage", "ArmFF");
                public static final PersistentShufflableDouble PSDArmTolerace = new PersistentShufflableDouble(.5,
                                "armThreshhold",
                                "ArmFF");
                public static final double[] kShoulderFF = { 0, .85, 3.2, 0.05 };
                public static PersistentShufflableDouble[] ShoulderFFPSDs = {
                                new PersistentShufflableDouble(0.0, "WShoulderKS", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WShoulderKG", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WShoulderKV", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WShoulderKA", "WAFV")
                };
                public static final double[] kElbowFF = { 0, .685, 1.8, 0.05 };
                public static PersistentShufflableDouble[] ElbowFFPSDs = {
                                new PersistentShufflableDouble(0.0, "WElbowKS", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WElbowKG", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WElbowKV", "WAFV"),
                                new PersistentShufflableDouble(0.0, "WElbowKA", "WAFV")
                };
                public static final double[] kShoulderPID = { 0.28, 0.05, 0.003 };
                public static PersistentShufflableDouble[] ShoulderPIDPSDs = {
                                new PersistentShufflableDouble(0, "WShoulderP", "WAFV"),
                                new PersistentShufflableDouble(0, "WShoulderI", "WAFV"),
                                new PersistentShufflableDouble(0, "WShoulderD", "WAFV")
                };
                public static final double[] kElbowPID = { 0.13, 0.01, 0.0015 };
                public static PersistentShufflableDouble[] ElbowPIDPSDs = {
                                new PersistentShufflableDouble(0, "WElbowP", "WAFV"),
                                new PersistentShufflableDouble(0, "WElbowI", "WAFV"),
                                new PersistentShufflableDouble(0, "WElbowD", "WAFV")
                };
                public static final int kProtectorUp = 7;
                public static final int kProtectorDown = 6;
                public static final double KStowPressVelocity = -.5;
                public static final int KTrajectorySteps = 5;
                public static final double kArmTolerance = 5; // 1;// in degrees

                public static final double[] kShoulderSafezone = { -140, 20 };// lower limit is index 0, upper limit is
                                                                              // index 1
                public static final double[] kElbowSafezone = { -10, 158 };// lowe limit is index 0, upper limit is
                                                                           // index 1
                public static final double kArmMotorDeadband = .1;
                public static final double kShoulderLength = 0.762;// meters //30 in
                // notes: the origin point is 48 inches off the ground
                // safe volume within robot frame is relatively small, just above the platform
                // no need to have elbow go negative angles
                // actuate elbow first
                public static final double kElbowLength = 0.762;// meters, the superior measuring system //30 in
                public static final double kArmLengths[] = { kShoulderLength, kElbowLength };
                public static final double kShoulderEncoderConversion = 360;// 360 degrees per rotation
                public static final double kElbowEncoderConversion = 360;// 360 degrees per rotation
                public static final int kShoulderMotorID = 31;
                public static final int kElbowMotorID = 32;
                public static final int kShoulderEncoderPin = 1;
                public static final int kElbowEncoderPin = 0;
                public static final double kShoulderEncOffset = -141.2;
                public static final double kElbowEncOffset = 112;
                public static final double kInitialShoulderTarget = -112.3;
                public static final double kInitialElbowTarget = 97.6;

                public static final PersistentShufflableDouble PSDShoulderOffset = new PersistentShufflableDouble(96,
                                "shoulderOffset", "ArmFF"); // degrees offset
                public static final PersistentShufflableDouble PSDElbowOffset = new PersistentShufflableDouble(-124,
                                "elbowOffset",
                                "ArmFF"); // degrees offset
                public static final double kShoulderMiddleAngleThreshold = -45;// the angle that is between the two
                                                                               // limit
                                                                               // switches
                                                                               // for the shoulder
                public static final int kShoulderLimitSwitch = 2;

                public static double squareOf(double i) {
                        return Math.pow(i, 2);
                }

                public static final class ArmSequences {
                        public static final double kElbowWristedPickup = 62.5;
                        public static final double kElbowPickupNormal = 67;
                        public static final double[] kStowAngles = { -115.2, 115 };
                        // pickup sequences
                        public static ArmKeyframe[] kfseqCubeStowToCubePickup = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        12),
                                        // new ArmKeyframe(new double[] { -115.2, 115 }, armKeyFrameStates.intermediary,
                                        // 5),
                                        new ArmKeyframe(new double[] { -75, 110 }, armKeyFrameStates.intermediary,
                                                        12),
                                        new ArmKeyframe(new double[] { -85, 67 }, armKeyFrameStates.intermediary,
                                                        12),
                                        new ArmKeyframe(new double[] { -81.8, kElbowPickupNormal },
                                                        armKeyFrameStates.pickupGround,
                                                        12),
                        };
                        // stowing sequences
                        public static ArmKeyframe[] kfseqCubePickuptoCubeStow = {

                                        // new ArmKeyframe(new double[] { -55.5, 118 },
                                        // armKeyFrameStates.intermediary,7),
                                        new ArmKeyframe(new double[] { -84, 75 }, armKeyFrameStates.intermediary,
                                                        12),
                                        new ArmKeyframe(new double[] { -75, 110 }, armKeyFrameStates.intermediary,
                                                        12),
                                        // new ArmKeyframe(new double[] { -90, 80 }, armKeyFrameStates.intermediary,
                                        // 12),
                                        new ArmKeyframe(new double[] { -115.2, 113 }, armKeyFrameStates.stowed,
                                                        12)
                        };
                        public static ArmKeyframe[] kfseqConeHightoCubeStow = {
                                        // new ArmKeyframe(new double[] { 10, 15 }, armKeyFrameStates.intermediary, 50),
                                        new ArmKeyframe(new double[] { -20, 83 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -52.5, 105 }, armKeyFrameStates.intermediary,
                                                        50),
                                        // new ArmKeyframe(new double[] { -90, 113 }, armKeyFrameStates.intermediary,
                                        // 50),
                                        new ArmKeyframe(new double[] { -115.2, 113 }, armKeyFrameStates.stowed, 50),
                                        // new ArmKeyframe(new double[] { -115.2, 104 }, armKeyFrameStates.stowed, 50)

                        };
                        public static ArmKeyframe[] kfseqConeMidtoCubeStow = {
                                        // new ArmKeyframe(new double[] { -55.5, 118 }, armKeyFrameStates.intermediary,
                                        // 7),
                                        // new ArmKeyframe(new double[] { -90, 113 }, armKeyFrameStates.intermediary,
                                        // 50),
                                        new ArmKeyframe(new double[] { -115.2,
                                                        113 }, armKeyFrameStates.stowed, 100)
                        };
                        public static ArmKeyframe[] kfseqCubehightoCubeStow = {
                                        new ArmKeyframe(new double[] { -54.5, 90 }, armKeyFrameStates.intermediary, 50),
                                        new ArmKeyframe(new double[] { -115.2, 113 }, armKeyFrameStates.stowed, 100)
                        };
                        public static ArmKeyframe[] kfseqCubeMidtoCubeStow = {
                                        new ArmKeyframe(new double[] { -91, 130 }, armKeyFrameStates.intermediary, 50),
                                        // new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                        // 7),
                                        // new ArmKeyframe(new double[] { -115.2, 120 }, armKeyFrameStates.stowed, 15)
                                        new ArmKeyframe(new double[] { -115.2, 113 }, armKeyFrameStates.stowed, 100)
                        };
                        // scoring sequences
                        public static ArmKeyframe[] kfseqCubeStowToCubeMid = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        new ArmKeyframe(new double[] { -91, 135 }, armKeyFrameStates.intermediary, 50),

                                        new ArmKeyframe(new double[] { -75, 130 }, armKeyFrameStates.intermediary, 50),

                                        new ArmKeyframe(new double[] { -64, 98 }, armKeyFrameStates.scoreCubeMid, 50),
                        };
                        public static ArmKeyframe[] kfseqCubeStowToCubeHigh = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        new ArmKeyframe(new double[] { -91, 135 }, armKeyFrameStates.intermediary, 50),
                                        new ArmKeyframe(new double[] { -20, 115 }, armKeyFrameStates.intermediary, 50),
                                        new ArmKeyframe(new double[] { -18, 40 }, armKeyFrameStates.scoreCubeHigh, 50),
                        };
                        public static ArmKeyframe[] kfseqConeStowToConeMid = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        new ArmKeyframe(new double[] { -55, 122 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -53.5, 98 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -59.6, 79 }, armKeyFrameStates.scoreConeMid, 50),

                        };
                        public static ArmKeyframe[] kfseqConeStowToConeHigh = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        // new ArmKeyframe(new double[] { -99, 119 }, armKeyFrameStates.intermediary,
                                        // 8),
                                        new ArmKeyframe(new double[] { -52.5, 125 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -20, 83 }, armKeyFrameStates.intermediary,
                                                        50),

                                        new ArmKeyframe(new double[] { 10, 20 }, armKeyFrameStates.intermediary, 50),
                                        new ArmKeyframe(new double[] { 0, 0 }, armKeyFrameStates.scoreConeHigh, 50),

                        };
                        public static ArmKeyframe[] kfseqConeStowToConeHighHover = {
                                        new ArmKeyframe(new double[] { -115.2, 135 }, armKeyFrameStates.intermediary,
                                                        3),
                                        // new ArmKeyframe(new double[] { -99, 135 }, armKeyFrameStates.intermediary,
                                        // 50),
                                        new ArmKeyframe(new double[] { -52.5, 155 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -20, 83 }, armKeyFrameStates.intermediary,
                                                        50),

                                        new ArmKeyframe(new double[] { 10, 20 }, armKeyFrameStates.hoverConeHigh, 50),
                        };
                        public static ArmKeyframe[] kfseqConeStowToConeMidHover = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        new ArmKeyframe(new double[] { -55, 122 }, armKeyFrameStates.intermediary,
                                                        50),
                                        new ArmKeyframe(new double[] { -53.5, 98 }, armKeyFrameStates.hoverConeMid,
                                                        50),
                        };
                        public static ArmKeyframe[] kfseqConeHighHoverToConeHighScore = {
                                        new ArmKeyframe(new double[] { 0, 0 }, armKeyFrameStates.scoreConeHigh, 25),
                        };
                        public static ArmKeyframe[] kfseqConeMidHoverToConeMidScore = {
                                        new ArmKeyframe(new double[] { -59.6, 79 }, armKeyFrameStates.scoreConeMid, 50),
                        };
                        public static ArmKeyframe[] kfseqConeMidScoreToConeMidHover = {
                                        new ArmKeyframe(new double[] { -53.5,
                                                        98 }, armKeyFrameStates.hoverConeMid,
                                                        50),
                        };
                        public static ArmKeyframe[] kfseqConeHighScoreToConeHighHover = {
                                        new ArmKeyframe(new double[] { 10, 20 }, armKeyFrameStates.hoverConeHigh, 25),
                        };

                        // chute sequences
                        public static ArmKeyframe[] kfseqStowtoChute = {
                                        new ArmKeyframe(new double[] { -115.2, 110 }, armKeyFrameStates.intermediary,
                                                        3),
                                        // new ArmKeyframe(new double[] { -55, 122 }, armKeyFrameStates.intermediary,
                                        // 50),
                                        // new ArmKeyframe(new double[] { -53.5, 98 }, armKeyFrameStates.intermediary,
                                        // 50),
                                        new ArmKeyframe(new double[] { -104, 136 }, armKeyFrameStates.pickupChute, 50),
                        };
                        public static ArmKeyframe[] kfseqChutetoStow = {
                                        new ArmKeyframe(new double[] { -115.2,
                                                        113 }, armKeyFrameStates.stowed, 50)
                        };
                }
        }

        public static final class LEDConstants {
                public static int kLEDPort = 9;
                public static int kLEDLength = 146;
        }
}