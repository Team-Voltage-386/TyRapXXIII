package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;

public class Balance extends CommandBase {
    private double balanceTarget = 2;
    private final Drivetrain dt;
    private Timer time = new Timer();
    private final PID pid = new PID(0.05, 0, 0.22);
    

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        time.reset();
        time.start();
        
        // //makes all wheels face the same direction (direction of target pos is pointed when called)
        // LeftFront.resetWheel();
        // RightFront.resetWheel();
        // LeftRear.resetWheel();
        // RightFront.resetWheel();

        // //stop wheels from turning
        // LeftFront.steerMotor.set(0);
        // RightFront.steerMotor.set(0);
        // LeftRear.steerMotor.set(0);
        // RightRear.steerMotor.set(0);
    }

    //maybe this'll work?
    @Override
    public void execute() {
        //assigns ypr vals
        SmartDashboard.putNumber("Pigeon Pitch", dt.ypr[2]);
        //balancing
        dt.xDriveTarget = -pid.calc(balanceTarget - dt.ypr[2]);
    }

    @Override
    public boolean isFinished() {
        //if the angle is straighter than the tolerance, it kills the command
        if(Math.abs(dt.ypr[2]) < balanceTarget) {
            
            if(Math.abs(dt.ypr[2]) < balanceTarget && time.hasElapsed(2)) {
                return true;
            }
            else{
                time.reset();
                return false;
            }
        }
        return false;
        //ignore this <-
    }

    @Override
    public void end(boolean interrupted) {
        balanceTarget = 0;
        System.out.println("Balancing done.");
        time.stop();
    }

}
