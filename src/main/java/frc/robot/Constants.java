// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.PIDShufflable;
import frc.robot.utils.PersistentShufflableDouble;

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
    public static final Joystick kDriver = new Joystick(0);
    // public static final Joystick kManipulator = new Joystick(1);

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

    public static final double driveTolerance = 0.05;
    public static final double headingTolerance = 90;

    public static final double[] kAutoPositionPID = { 1, 2.5, 0.2 };
    public static final double[] kAutoHeadingPID = { 2, 0.1, 0.1 };
  }

  /** Can IDs, PID values, ect. */
  public static final class DriveConstants {

    // public static final double kMaxRotSpeed = 180; // should be in degrees per
    // second
    // public static final double kMaxDriveSpeed = 4; // should be in meters per
    // second
    public static PersistentShufflableDouble kMaxRotSpeed = new PersistentShufflableDouble(180, "maxRotationSpeed");
    public static PersistentShufflableDouble kMaxDriveSpeed = new PersistentShufflableDouble(4, "maxDriveSpeed");
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
    public static final SwerveModule LeftFront = new SwerveModule(14, 18, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 24, 0.365125, -0.263525, 84.1, "LF");
    public static final SwerveModule RightFront = new SwerveModule(11, 15, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 21, 0.365125, 0.263525, 213.15, "RF");
    public static final SwerveModule LeftRear = new SwerveModule(13, 17, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 23, -0.365125, -0.263525, 126.25, "LR");
    public static final SwerveModule RightRear = new SwerveModule(12, 16, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 22, -0.365125, 0.263525, 292, "RR");
  }

  public static final class SmoothingConstants {
    // public static final double kAccelerationSmoothFactor = .2;
    public static PersistentShufflableDouble kAccelerationSmoothFactor = new PersistentShufflableDouble(.2,
        "driveAccelSmooth");
    // public static final double kRotationAccelerationSmoothFactor = .5;
    public static PersistentShufflableDouble kRotationAccelerationSmoothFactor = new PersistentShufflableDouble(.5,
        "rotateAccelSmooth");

  }

  public static final class ArmConstants {
    public static final PersistentShufflableDouble PSDShoulderMaxPercentage = new PersistentShufflableDouble(.1,
        "armMaxSpeed", "Arm");
        public static final PersistentShufflableDouble PSDElbowMaxPercentage = new PersistentShufflableDouble(.1,
        "armMaxSpeed", "Arm");
    public static final PersistentShufflableDouble armThreshold = new PersistentShufflableDouble(.5, "armThreshhold",
        "Arm");

    public static final double[] kArmShoulderPID = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    public static final double[] kArmElbowPID = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    public static final double[] kShoulderSafezone = { -150, 0 };// lower limit is index 0, upper limit is index 1
    public static final double[] kElbowSafezone = { 0, 90 };// lowe limit is index 0, upper limit is index 1
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
    public static final PersistentShufflableDouble PSDShoulderOffset = new PersistentShufflableDouble(96,
        "shoulderOffset", "Arm"); // degrees offset
    public static final PersistentShufflableDouble PSDElbowOffset = new PersistentShufflableDouble(-124, "elbowOffset",
        "Arm"); // degrees offset
    public static final double kShoulderMiddleAngleThreshold = -45;// the angle that is between the two limit switches
                                                                   // for the shoulder
    public static final int kShoulderLimitSwitch = 2;

    public static double squareOf(double i) {
      return Math.pow(i, 2);
    }

    public static final class ArmSequences {

    }
  }
}