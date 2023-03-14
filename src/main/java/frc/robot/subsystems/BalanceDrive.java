package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Autonomous.Drive;
import frc.robot.subsystems.Drivetrain;

public class BalanceDrive extends SubsystemBase {
    private double x;
    private double y;
    private double h;
    private Drivetrain dt;
    private int Direction;
    private double ypr[] = new double[3];
    private boolean BalanceEnabled=false;
    
    public BalanceDrive(double X, Drivetrain DT) {
        x = X;
        y = 0;
        h = 180;
        dt = DT;
        Drivetrain.IMU.getYawPitchRoll(ypr);
        if (ypr[2]>0)
        {
            Direction=-1;
        }
        else
        {
            Direction=1;
        }
        BalanceEnabled=true;
    }
    
    @Override
    public void periodic() {
        Drivetrain.IMU.getYawPitchRoll(ypr);
        if(BalanceEnabled && Math.abs(ypr[2])<=2 && BalanceEnabled && DriverStation.isAutonomousEnabled())
        {
            new Drive(x*Direction, y, h, dt);
        }
        if (ypr[2]>0)
        {
            Direction=-1;
        }
        else
        {
            Direction=1;
        }
    }
    

}
