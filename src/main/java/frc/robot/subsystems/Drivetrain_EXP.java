package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain_EXP extends SubsystemBase {

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveLocations);;
  private SwerveDriveOdometry odometry;
  public SwerveModule[] swerveModules = new SwerveModule[4];
  private PigeonIMU gyro;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);
  private double startingX, startingY;
  private Pose2d pose;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  private SwerveModulePosition[] SwerveModPos;

  public Drivetrain_EXP(double startingX, double startingY) {
    this.startingX = startingX;
    this.startingY = startingY;
    gyro = new PigeonIMU(kIMUid); 

    SwerveModPos[0] = new SwerveModulePosition(0, new Rotation2d(Math.toDegrees(0)));
    SwerveModPos[1] = new SwerveModulePosition(0, new Rotation2d(Math.toDegrees(0)));
    SwerveModPos[2] = new SwerveModulePosition(0, new Rotation2d(Math.toDegrees(0)));
    SwerveModPos[3] = new SwerveModulePosition(0, new Rotation2d(Math.toDegrees(0)));

    swerveModules[0] = RightFront;
    swerveModules[1] = RightRear;
    swerveModules[2] = LeftRear;
    swerveModules[3] = LeftFront;
    
    odometry = new SwerveDriveOdometry(kinematics, rotation(), SwerveModPos);
    pose = new Pose2d(startingX,startingY, rotation());
    tab.getLayout("Odometry", BuiltInLayouts.kList).addNumber("Robot X", ()->x());
    tab.getLayout("Odometry", BuiltInLayouts.kList).addNumber("Robot Y", ()->y());
    tab.getLayout("Odometry", BuiltInLayouts.kList).addNumber("Robot Rotation",()->rotation().getDegrees());
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4);
    pose = odometry.update(rotation(), SwerveModPos);
  }

  public void reset(boolean zeroGyro){
    System.out.println("reset");
    if(zeroGyro) zero();
    odometry.resetPosition(rotation(), SwerveModPos, new Pose2d(startingX,startingY,rotation()));   
  }

  public void zero(){
    gyro.setFusedHeading(0.0);
  }

  public PigeonIMU getGyro(){
    return gyro;
  }
  
  public double x(){
    return pose.getX();
  }

  public double y(){
    return pose.getY();
  }

  public Rotation2d rotation(){
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }
}