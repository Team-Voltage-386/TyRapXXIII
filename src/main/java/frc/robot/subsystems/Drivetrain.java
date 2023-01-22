package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
    public double xDriveTarget = 0;
    public double yDriveTarget = 0;
    public double rotationTarget = 0;

    public double xPos = 0;
    public double yPos = 0;
    public double angle = 0;

    private double ypr[] = new double[3];

    public Pigeon2 IMU = new Pigeon2(kIMUid);

    private boolean wasEnabled = false;

    private Timer odometryTimer = new Timer();
    private double odoTimerLast = 0;

    public SwerveModule[] modules = { RightFront, RightRear, LeftRear, LeftFront };

    public Drivetrain() {
        this.init();
    }

    public void init() {
        odometryTimer.start();
        odoTimerLast = odometryTimer.get();
    }

    @Override
    public void periodic() {
        updateOdometry();
        if (Robot.inst.isEnabled()) {
            for (SwerveModule swerve : modules) {
                if (Math.abs(xDriveTarget) > 0.05 || Math.abs(yDriveTarget) > 0.05 || Math.abs(rotationTarget) > 1) {
                    double angleRad = Math.toRadians(angle);

                    double x = xDriveTarget;
                    double y = yDriveTarget;

                    double r = ((2 * Math.PI * swerve.distFromCenter) / 360) * rotationTarget; // rotation speed
                    double rAngle = swerve.angleFromCenter + angle + 90;
                    x += r * Math.cos(Math.toRadians(rAngle));
                    y += r * Math.sin(Math.toRadians(rAngle));

                    double xFin = (x * Math.cos(angleRad)) + (y * Math.sin(angleRad));
                    double yFin = (x * Math.cos(angleRad + (Math.PI / 2))) + (y * Math.sin(angleRad + (Math.PI / 2)));

                    swerve.targetSteer = Math.toDegrees(Math.atan2(yFin, xFin));
                    swerve.targetDrive = Math.sqrt(Math.pow(xFin, 2) + Math.pow(yFin, 2));
                } else {
                    swerve.targetDrive = 0;
                    swerve.drivePID.reset();
                    swerve.targetSteer = swerve.angleFromCenter;
                }

                swerve.drive();
            }

            wasEnabled = true;
        } else {
            if (wasEnabled)
                for (SwerveModule swerve : modules)
                    swerve.reset();
            wasEnabled = false;
        }

        updateWidget();
    }

    public double getRawHeading() {
        double y = ypr[0];
        while (y < 0)
            y += 360;
        while (y > 360)
            y -= 360;
        return -y;
    }

    public void setOffset(double offX, double offY) {
        for (SwerveModule swerve : modules)
            swerve.calcPosition(offX, offY);
    }

    public void resetFO() {
        IMU.setYaw(0);
    }

    private void updateOdometry() {
        IMU.getYawPitchRoll(ypr);
        angle = getRawHeading();

        if (Robot.inst.isEnabled()) {

            if (!wasEnabled) {
                odoTimerLast = odometryTimer.get();
            }

            double time = odometryTimer.get();
            odometryTimer.reset();
            odometryTimer.start();
            double deltaT = time - odoTimerLast;
            odoTimerLast = time;

            double xAdd = 0;
            double yAdd = 0;

            for (SwerveModule swerve : modules) {
                double aRad = Math.toRadians(angle + swerve.getEncoderPosition());
                double vel = swerve.driveMotor.getEncoder().getVelocity();
                xAdd += deltaT * (Math.cos(aRad) * vel);
                yAdd += deltaT * (Math.sin(aRad) * vel);
            }

            xPos += xAdd / modules.length;
            yPos += yAdd / modules.length;
        }
    }

    private static final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private static final GenericEntry xPosWidget = mainTab.add("X", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = mainTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();

    private void updateWidget() {
        xPosWidget.setDouble(xPos);
        yPosWidget.setDouble(yPos);
    }

}
