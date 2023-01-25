// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.apriltag;
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
  public static final class Limelightconstants{
    public static final int apriltagpipelineindex = 0;
    public static final int retroreflectivepipelineindex = 1;
  }
  /** Can IDs, PID values, ect. */
  public static final class DriveConstants {

    public static final double kMaxRotSpeed = 200; // should be in degrees per second
    public static final double kMaxDriveSpeed = 5; // should be in meters per second

    public static final int kIMUid = 2;
    public static final double[] kSwerveSteerPID = { 0.01, 0.0, 0.001 };
    public static final double[] kSwerveDrivePID = { 0.35, 2, 0.01 };
    public static final double kSwerveDriveEncConv = 0.0005;

    public static final SwerveModule LeftFront = new SwerveModule(14, 18, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 24, 0.36, -0.26, 84.1);
    public static final SwerveModule RightFront = new SwerveModule(11, 15, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 21, 0.36, 0.26, 213.15);
    public static final SwerveModule LeftRear = new SwerveModule(13, 17, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 23, -0.36, -0.26, 126.25);
    public static final SwerveModule RightRear = new SwerveModule(12, 16, kSwerveDriveEncConv, kSwerveSteerPID,
        kSwerveDrivePID, 22, -0.36, 0.26, 292);
  }

  public static final class Field{
    public static final apriltag tag1=new apriltag(1, true,true, 7.24310, -2.93659, .46272);
    public static final apriltag tag2=new apriltag(2, true,true, 7.24310,-1.26019, .46272);
    public static final apriltag tag3=new apriltag(3, true,true, 7.24310, 0.41621, .46272);
    public static final apriltag tag4=new apriltag(4, false,false, 7.90832, 2.74161, 0.695452);
    public static final apriltag tag5=new apriltag(5, false,true, -7.90832, 2.74161, 0.695452);
    public static final apriltag tag6=new apriltag(6, true,false, -7.24310, 0.41621, .46272);
    public static final apriltag tag7=new apriltag(7, true,false, -7.24310, -1.26019, .46272);
    public static final apriltag tag8=new apriltag(8, true,false, -7.24310, -2.93659, .46272);
    public static final apriltag[] tags = {tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8};

    public static apriltag closestGrid(double x,double y){
      apriltag result=null;
      double best=255;
      double tmp=0;
      for (apriltag i:tags){
        tmp=Math.pow(i.x-x,2)+Math.pow(i.y-y,2);
        if (i.isGrid&&tmp<best) {
          result=i;
          best=tmp;
        }
      }
      return result;
    }
  }

  public static final class AutoPilotConstants{
    public static final double moep=0.1;//margin of error, position
    public static final double moer=0.1;//margin of error, rotation
    public static final double adjustableXDist=0.5;

  }
}