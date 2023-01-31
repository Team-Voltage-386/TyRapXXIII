package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance extends CommandBase {
    private double balanceTarget = 2.5;
    private double robotBalance = 0;
    private double ypr[] = new double[3];
    public Pigeon2 pigeonIMU = new Pigeon2(kIMUid);
    private PIDController pidcont = new PIDController(kMaxDriveSpeed, kIMUid, kIMUid);

    public Balance() {
        
    }

    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        //makes all wheels face the same direction (direction of target pos is pointed when called)
        LeftFront.resetWheel();
        RightFront.resetWheel();
        LeftRear.resetWheel();
        RightFront.resetWheel();

        //stop wheels from turning
        LeftFront.steerMotor.set(0);
        RightFront.steerMotor.set(0);
        LeftRear.steerMotor.set(0);
        RightRear.steerMotor.set(0);
    }

    //maybe this'll work?
    @Override
    public void execute() {
        //assigns ypr vals
        ypr[1] = pigeonIMU.getPitch();
        //easier to read
        robotBalance = ypr[1];
        pigeonIMU.getYawPitchRoll(ypr);
        //balancing
        LeftFront.driveMotor.set(pidcont.calculate(balanceTarget - ypr[1]));
        RightFront.driveMotor.set(pidcont.calculate(balanceTarget - ypr[1]));
        LeftRear.driveMotor.set(pidcont.calculate(balanceTarget - ypr[1]));
        RightRear.driveMotor.set(pidcont.calculate(balanceTarget - ypr[1]));
    }

    @Override
    public boolean isFinished() {
        //if the angle is straighter than the tolerance, it kills the command
        if(Math.abs(robotBalance) < balanceTarget) {
            end(true);
            return true;
        }
        else return false;
    }

}
