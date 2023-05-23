package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
    public double xDriveTarget = 0;
    public double yDriveTarget = 0;
    public double rotationTarget = 0;
    public double speed = 0;

    public double xPos = 0;
    public double yPos = 0;
    public double angle = 0;

    public double ypr[] = new double[3];

    public Pigeon2 IMU = new Pigeon2(kIMUid);

    private boolean wasEnabled = false;

    private Timer odometryTimer = new Timer();
    private long odoTimerLast = 0;
    public boolean doFieldOrientation = true;
    public Pose2d pose;

    public SwerveModule[] modules = { RightFront, RightRear, LeftRear, LeftFront };

    public Drivetrain() {
        this.init();
    }

    public void init() {
        odoTimerLast = System.currentTimeMillis();
        resetFO();
        doFieldOrientation = true;
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

                    targetSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(x, 2));

                    double r = ((2 * Math.PI * swerve.distFromCenter) / 360) * rotationTarget; // rotation speed

                    double rAngle = swerve.angleFromCenter + 90;
                    if (doFieldOrientation) rAngle += angle;


                    x += r * Math.cos(Math.toRadians(rAngle));
                    y += r * Math.sin(Math.toRadians(rAngle));

                    double xFin = x;
                    double yFin = y;

                    if (doFieldOrientation) {
                        xFin = (x * Math.cos(angleRad)) + (y * Math.sin(angleRad));
                        yFin = (x * Math.cos(angleRad + (Math.PI / 2))) + (y * Math.sin(angleRad + (Math.PI / 2)));
                    }

                    swerve.targetSteer = Math.toDegrees(Math.atan2(yFin, xFin));
                    swerve.targetDrive = Math.sqrt(Math.pow(xFin, 2) + Math.pow(yFin, 2));
                } else {
                    swerve.targetDrive = 0;
                    swerve.drivePID.reset();
                    swerve.targetSteer = swerve.angleFromCenter + 90;// circle lock is add 90, x lock is add 0
                    // swerve.angleFromCenter + 90
                }

                swerve.drive();
                // swerve.updateShufflables();
            }

            wasEnabled = true;
        } else {
            if (wasEnabled)
                for (SwerveModule swerve : modules)
                    swerve.reset();

            wasEnabled = false;
            for (SwerveModule swerve : modules) // diagnosing tool, delete later
                swerve.updateWidget();// diagnosing tool, delete later
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
        IMU.setYaw(180);
    }

    public void feedBotPose(double x, double y, double FieldOrientation) {
        xPos = x;
        yPos = y;
        IMU.setYaw(FieldOrientation);
    }

    public void resetFO(double a) {
        IMU.setYaw(a);
    }

    private void updateOdometry() {
        IMU.getYawPitchRoll(ypr);
        angle = getRawHeading();

        if (Robot.inst.isEnabled()) {

            if (!wasEnabled) {
                odoTimerLast = System.currentTimeMillis();
            }

            long thisTime = System.currentTimeMillis();

            double deltaT = (thisTime - odoTimerLast);
            deltaT /= 1000;
            odoTimerLast = thisTime;

            double xAdd = 0;
            double yAdd = 0;

            double xSpeed = 0;
            double ySpeed = 0;

            for (SwerveModule swerve : modules) {
                double aRad = Math.toRadians(angle + swerve.getEncoderPosition());
                double vel = swerve.driveMotor.getEncoder().getVelocity();
                xAdd += deltaT * (Math.cos(aRad) * vel);
                yAdd += deltaT * (Math.sin(aRad) * vel);
                xSpeed += Math.cos(aRad) * vel;
                ySpeed += Math.sin(aRad) * vel;
            }

            xAdd /= modules.length;
            yAdd /= modules.length;
            xSpeed /= modules.length;
            ySpeed /= modules.length;

            xPos += xAdd;
            yPos += yAdd;

            speed = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
        }
        pose = new Pose2d(xPos, yPos, getHeadingRotation2d());
    }

    public double distanceTo(double x, double y) {
        return Math.sqrt(Math.pow(x - xPos, 2) + Math.pow(y - yPos, 2));
    }

    //*zeros heading */
    public void zeroHeading() {
        IMU.setYaw(0);
    }
    
    //*get heading of robot in degrees*/
    public double getHeading() {
        return Math.IEEEremainder((IMU.getYaw()), 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetOdometry(Pose2d p) {
        pose = p;
    }

    public double getHeadingError(double h) {
        double res = h - getRawHeading() - 180;
        if (360-Math.abs(res)<10)
        {
            if (res<0)
            {
                //Robot wants to do negative 360
                res=360+res;
            }
            else 
            {
                //Robot wants to do positive 360
                res=360-res;
            }
        }
        while (angle > 180)
            angle -= 360;
        while (angle < 180)
            angle += 360;
        return res;
    }

    private static final ShuffleboardTab OdomeTab = Shuffleboard.getTab("Odometry");
    private static final ShuffleboardTab speedTab = Shuffleboard.getTab("Speed");
    private static final GenericEntry xPosWidget = OdomeTab.add("X", 0).withPosition(0, 0).withSize(1, 1).getEntry();
    private static final GenericEntry yPosWidget = OdomeTab.add("Y", 0).withPosition(1, 0).withSize(1, 1).getEntry();
    private static final GenericEntry rotationWidget = OdomeTab.add("yaw", 0).getEntry();
    private static final GenericEntry pitchWidget = OdomeTab.add("pitch", 0).getEntry();
    private static final GenericEntry rollWidget = OdomeTab.add("roll", 0).getEntry();
    private static final GenericEntry targetSpeedWidget = speedTab.add("target", 0).getEntry();
    private static final GenericEntry speedWidget = speedTab.add("current", 0).getEntry();
    private static final GenericEntry RotateWidget = speedTab.add("Rotate", 0).getEntry();
    private double targetSpeed = 0;

    private void updateWidget() {
        xPosWidget.setDouble(xPos);
        yPosWidget.setDouble(yPos);
        rotationWidget.setDouble(ypr[0]);
        pitchWidget.setDouble(ypr[2]);
        rollWidget.setDouble(ypr[1]);
        targetSpeedWidget.setDouble(targetSpeed);
        speedWidget.setDouble(speed);
        RotateWidget.setDouble(getHeadingError(180));
    }

}
