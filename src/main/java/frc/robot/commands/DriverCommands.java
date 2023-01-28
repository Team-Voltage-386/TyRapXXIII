package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.PID;
import frc.robot.utils.apriltag;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.Field.*;
import static frc.robot.utils.Flags.*;

import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.AutoPilotConstants.*;
import static frc.robot.Constants.Limelightconstants.*;

public class DriverCommands extends CommandBase {

    private Drivetrain driveTrain;
    
    private Limelight limelight;
    private apriltag target=null;
    private double facingtoscore;
    private double deadband=0.1;
    private PID autoRPID;
    private PID autoXPID;
    private PID autoYPID;

    public DriverCommands(Drivetrain DT, Limelight LL) {
        driveTrain = DT;
        limelight = LL;
        addRequirements(driveTrain);
        addRequirements(limelight);
        facingtoscore=270;//set to 0 or 180
        autoRPID=new PID(kAutoRotationPID[0],kAutoRotationPID[1],kAutoRotationPID[2]);
        autoXPID=new PID(kAutoDriveXPID[0],kAutoDriveXPID[1],kAutoDriveXPID[2]);
        autoYPID=new PID(kAutoDriveYPID[0],kAutoDriveYPID[1],kAutoDriveYPID[2]);
    }

    @Override
    public void initialize() {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    @Override
    public void execute() {
        // System.out.println(HumanDriverControl);
        HumanDriverControl=Math.abs(kDriver.getRawAxis(kLeftTrigger))<deadband;

        if(HumanDriverControl){
            driveTrain.yDriveTarget = kDriver.getRawAxis(kLeftHorizontal) * kMaxDriveSpeed;
            driveTrain.xDriveTarget = kDriver.getRawAxis(kLeftVertical) * kMaxDriveSpeed;
            driveTrain.rotationTarget = -kDriver.getRawAxis(kRightHorizontal) * kMaxRotSpeed;

            if (kDriver.getRawButtonPressed(kLeftBumper))
                driveTrain.setOffset(-0.65, 0);
            else if (kDriver.getRawButtonReleased(kLeftBumper))
                driveTrain.setOffset(0, 0);

            if (kDriver.getRawButtonPressed(kRightBumper)){
                if(limelight.apriltagmode()&&limelight.apriltagsAvailable())
                    driveTrain.setFO(limelightYawToDriveTrainYaw());
                else
                    driveTrain.resetFO();
            }
        }

        if (kDriver.getRawButtonPressed(kRightBumper)&&limelight.apriltagmode()&&limelight.apriltagsAvailable()){
            driveTrain.setFO(limelightYawToDriveTrainYaw());

        }
    
    //the manipulator will set the targetX and targetY using controller (logic to get coordinates from button input)

        if(limelight.apriltagmode() && Math.abs(kDriver.getRawAxis(kLeftTrigger))>deadband&&limelight.apriltagsAvailable()){
            HumanDriverControl=false;
            if(target==null) {target=closestGrid(limelight.getPose()[0],limelight.getPose()[1]);}
            driveTrain.setFO(limelightYawToDriveTrainYaw());
            driveToTarget(6.25,target.y,facingtoscore,limelight.getPose()[0],limelight.getPose()[1],driveTrain.getRawHeading());
        } 
        if(limelight.retroreflectivemode() && Math.abs(kDriver.getRawAxis(kLeftTrigger))>deadband){
            HumanDriverControl=false;
            driveToTarget(0, 0, facingtoscore, 0, 1*Math.atan(Math.toRadians(limelight.tx())) , driveTrain.getRawHeading());//the value tiimes arctan should be a constant (variable of sorts)
        } 
        if(kDriver.getRawButtonPressed(kX)){
            if(limelight.apriltagmode())limelight.setPipeline(retroreflectivepipelineindex);
            else if(limelight.retroreflectivemode())limelight.setPipeline(apriltagpipelineindex);
        }
    }
    public double limelightYawToDriveTrainYaw(){
        return limelight.getPose()[5]-90;
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    public void driveToTarget(double targetX, double targetY, double targetRot, double bpx, double bpy, double bpr){
        driveTrain.rotationTarget=autoRPID.calc(targetRot-bpr);
        driveTrain.xDriveTarget=autoXPID.calc(targetX-bpx);
        driveTrain.yDriveTarget=autoYPID.calc(targetY-bpy);
    }
    public double adjustableX(double targetX){
        double result=Math.abs(targetX)-adjustableXDist;
        result=6.25;
        return result;
    }
    @Override
    public void end(boolean interrupted) {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

}
