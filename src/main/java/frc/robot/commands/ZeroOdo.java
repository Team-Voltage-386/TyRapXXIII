package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ZeroOdo extends CommandBase {

    boolean done = false;
    double x;
    double y;
    double a;
    Drivetrain dt;

    public ZeroOdo(double X, double Y, double A, Drivetrain DT) {
        x =X;
        y = Y;
        a = A;
        dt = DT;
    }

    @Override
    public void initialize() {
        dt.xPos = x;
        dt.yPos = y;
        dt.resetFO(a);
        done = true;
    }

    public boolean isFinished() {
        return done;
    }
    
}
