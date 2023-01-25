package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.apriltag;

import static frc.robot.utils.Flags.*;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.Field.*;
import static frc.robot.Constants.AutoPilotConstants.*;
import static frc.robot.Constants.Limelightconstants.*;

public class AutoPilotCommands extends CommandBase {

    private Drivetrain driveTrain;
    private Limelight limelight;
    private apriltag target=null;
    private double facingtoscore=0.0;

    public AutoPilotCommands(Drivetrain DT, Limelight LL) {
        driveTrain = DT;
        limelight=LL;//[x,y,z, rotationx,y,z]
        facingtoscore=0;//set to 0 or 180
    }

    @Override
    public void initialize() {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    @Override
    public void execute() {
        if (kDriver.getRawButtonPressed(kRightBumper)&&limelight.apriltagsAvailable())
            driveTrain.setFO(limelight.getPose()[5]);
        
        //the manipulator will set the targetX and targetY using controller (logic to get coordinates from button input)

        if(limelight.apriltagmode() && kDriver.getRawAxis(kLeftTrigger)>0.5&&limelight.apriltagsAvailable()){
            HumanDriverControl=false;
            if(target==null) {target=Constants.Field.closestGrid(limelight.getPose()[0],limelight.getPose()[1]);}
            driveToTarget(adjustableX(target.x),target.y,facingtoscore,limelight.getPose()[0],limelight.getPose()[1],limelight.getPose()[5]);
        } else{HumanDriverControl=true;}
        if(limelight.retroreflectivemode() && kDriver.getRawAxis(kLeftTrigger)>0.5){
            HumanDriverControl=false;
            driveToTarget(0, 0, facingtoscore, limelight.tx(), 0, driveTrain.getRawHeading());
        }
        if(kDriver.getRawButtonPressed(kX)){
            if(limelight.apriltagmode())limelight.setPipeline(retroreflectivepipelineindex);
            if(limelight.retroreflectivemode())limelight.setPipeline(apriltagpipelineindex);
        }
       
    }
    public void driveToTarget(double targetX, double targetY, double targetRot, double bpx, double bpy, double bpr){
        double tempX=kMaxDriveSpeed;
        double tempY=kMaxDriveSpeed;
        double tempRot=kMaxRotSpeed;
        if(Math.abs(bpx-targetX)<moep){tempX=0;}
        else if(bpx>targetX){tempX*=-1;}
        if(Math.abs(bpy-targetY)<moep){tempY=0;}
        else if(bpy>targetY){tempY*=-1;}
        if(Math.abs(bpr-targetRot)<moer){tempRot=0;}
        else if(bpr>targetRot){tempRot*=-1;}
        driveTrain.xDriveTarget=tempX;
        driveTrain.yDriveTarget=tempY;
        driveTrain.rotationTarget=tempRot;
        if (tempRot==0&&tempY==0&&tempX==0){target=null;}
    }
    public double adjustableX(double targetX){
        double result=Math.abs(targetX)-adjustableXDist;
        return result;
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.xDriveTarget = 0;
        driveTrain.yDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

}
