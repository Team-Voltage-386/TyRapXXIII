package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PID;

public class Balance extends CommandBase {
    private double balanceTarget = 2.5;
    private final Drivetrain DT;
    private double ypr[] = new double[3];
    public Pigeon2 pigeonIMU = new Pigeon2(kIMUid);
    private PID pid = new PID(0, 0, 0);

    public Balance(Drivetrain dt) {
        DT = dt;
    }

    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        
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
        ypr[1] = pigeonIMU.getPitch();
        pigeonIMU.getYawPitchRoll(ypr);
        //balancing
        DT.xDriveTarget = pid.calc(balanceTarget - ypr[1]);
    }

    @Override
    public boolean isFinished() {
        //if the angle is straighter than the tolerance, it kills the command
        if(Math.abs(ypr[1]) < balanceTarget) {
            end(true);
            return true;
        }
        else return false;
    }

}
