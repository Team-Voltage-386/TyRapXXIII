package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Limelightconstants;
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
    private double deadband=0.1;

    public DriverCommands(Drivetrain DT, Limelight LL) {
        driveTrain = DT;
        limelight=LL;
        addRequirements(limelight);
        addRequirements(driveTrain);        
    }

    @Override
    public void initialize() {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

    @Override
    public void execute() {
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
                    driveTrain.setFO(limelight.limelightYawToDriveTrainYaw());
                else
                    driveTrain.resetFO();
            }
        }

        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        driveTrain.yDriveTarget = 0;
        driveTrain.xDriveTarget = 0;
        driveTrain.rotationTarget = 0;
    }

}
