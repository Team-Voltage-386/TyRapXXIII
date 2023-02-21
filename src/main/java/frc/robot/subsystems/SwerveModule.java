package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.PIDShufflable;
import frc.robot.utils.PIDShufflableLinked;
import frc.robot.utils.PersistentShufflableDouble;

import static frc.robot.utils.mapping.*;
import static frc.robot.Constants.DriveConstants;

public class SwerveModule {

    public final CANSparkMax steerMotor;
    public final CANSparkMax driveMotor;
    public final CANCoder encoder;
    public final PIDShufflableLinked steerPID;
    public final PIDShufflableLinked drivePID;
    public final double x;
    public final double y;
    public final double encoderOffs;
    public double angleFromCenter;
    public double distFromCenter;

    public double targetSteer = 0;
    public double targetDrive = 0;

    public int driveMult = 1;

    public int swerveModuleID;
    public static int swerveModuleCount = 0;
    public final ShuffleboardTab mainTab;
    public final GenericEntry steerMotorCurrentWidget;
    public final GenericEntry driveMotorCurrentWidget;
    public final GenericEntry driveMotorSetWidget;
    public final GenericEntry posiitonWidget;

    public SwerveModule(int STEERMOTOR, int DRIVEMOTOR, double driveConversion, PersistentShufflableDouble[] steerPIDPSD,
            PersistentShufflableDouble[] drivePIDPSD, int encoderID, double X, double Y, double ENCOFFS, String SwerveModuleName) {
        swerveModuleID = swerveModuleCount;

        steerPID = new PIDShufflableLinked(steerPIDPSD[0], steerPIDPSD[1], steerPIDPSD[2]);
        drivePID = new PIDShufflableLinked(drivePIDPSD[0], drivePIDPSD[1], drivePIDPSD[2]);
        steerMotor = new CANSparkMax(STEERMOTOR, MotorType.kBrushless);
        driveMotor = new CANSparkMax(DRIVEMOTOR, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(driveConversion);
        driveMotor.getEncoder().setVelocityConversionFactor(driveConversion);
        encoder = new CANCoder(encoderID);

        x = X;
        y = Y;
        encoderOffs = ENCOFFS;

        this.calcPosition(0, 0);

        mainTab = Shuffleboard.getTab("Main");
        steerMotorCurrentWidget = mainTab.add("steerMotor" + SwerveModuleName, 0).withPosition(5, swerveModuleID)
                .withSize(1, 1)
                .getEntry();
        driveMotorCurrentWidget = mainTab.add("drive motor" + SwerveModuleName, 0).withPosition(6,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        posiitonWidget = mainTab.add("orientation" + SwerveModuleName, 0).withPosition(7,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        driveMotorSetWidget = mainTab.add("dmSET" + SwerveModuleName, 0).withPosition(8, swerveModuleID).withSize(1, 1)
                .getEntry();
        swerveModuleCount++;

        updateShufflables();
    }

    public double getEncoderPosition() {
        return encoder.getAbsolutePosition() - encoderOffs;
    }

    public void calcPosition(double offX, double offY) {
        distFromCenter = Math.sqrt(Math.pow(x + offX, 2) + Math.pow(y + offY, 2));
        angleFromCenter = Math.toDegrees(Math.atan2(y + offY, x + offX));
    }

    private double getSwerveHeadingError() {
        double res = targetSteer - getEncoderPosition();
        while (res < -180)
            res += 360;
        while (res > 180)
            res -= 360;
        if (Math.abs(res) > 90) {
            driveMult = -1;
            if (res > 0)
                res -= 180;
            else
                res += 180;
        } else {
            driveMult = 1;
        }
        return res;
    }

    public void drive() {
        updateWidget();

        steerMotor.set(steerPID.calc(getSwerveHeadingError()));

        driveMotor.set(mapValue(getSwerveHeadingError(), 0, 180, 1, 0)
                * drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity()));
    }

    public void reset() {
        steerPID.reset();
        drivePID.reset();
        steerMotor.set(0);
        driveMotor.set(0);
    }

    private void updateWidget() {
        steerMotorCurrentWidget.setDouble(steerMotor.getOutputCurrent());
        driveMotorCurrentWidget.setDouble(driveMotor.getOutputCurrent());
        driveMotorSetWidget.setDouble(mapValue(Math.abs(getSwerveHeadingError()), 0, 180, 1, 0)
                * (drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity())));
        posiitonWidget.setDouble(getEncoderPosition());
    }

    public void updateShufflables() {
        if (steerPID.detectChange()) {
            steerPID.shuffleUpdatePID();
        }
        if (drivePID.detectChange()) {
            drivePID.shuffleUpdatePID();
        }
    }

}
