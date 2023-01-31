package frc.robot.commands.Autonomous;

import com.ctre.phoenix.sensors.Pigeon2;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
    private double balanceTarget = 2.5;
    private double robotBalance = 0;
    private final Drivetrain dt;
    private double ypr[] = new double[3];
    public Pigeon2 pigeonIMU = new Pigeon2(kIMUid);

    public Balance(Drivetrain DT) {
        dt = DT;
    }

    @Override
    public void initialize() {
        System.out.println("Balance Starting");
        pigeonIMU.setPitch
    }

    @Override
    public void execute() {
        pigeonIMU.getYawPitchRoll(ypr);
    }

    @Override
    public boolean isFinished() {
        if(balanceTarget - )
    }

}
