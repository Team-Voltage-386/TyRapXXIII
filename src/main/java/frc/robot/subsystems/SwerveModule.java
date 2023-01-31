package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.PID;

public class SwerveModule {

    public final CANSparkMax steerMotor;
    public final CANSparkMax driveMotor;
    public final CANCoder encoder;
    public final PID steerPID;
    public final PID drivePID;
    public final double x;
    public final double y;
    public final double encoderOffs;
    public double angleFromCenter;
    public double distFromCenter;

    public double targetSteer = 0;
    public double targetDrive = 0;

    public int driveMult = 1;

    public SwerveModule(int STEERMOTOR, int DRIVEMOTOR, double driveConversion, double[] steerPIDValue,
            double[] drivePIDValue, int encoderID, double X, double Y, double ENCOFFS) {
        steerMotor = new CANSparkMax(STEERMOTOR, MotorType.kBrushless);
        driveMotor = new CANSparkMax(DRIVEMOTOR, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(driveConversion);
        driveMotor.getEncoder().setVelocityConversionFactor(driveConversion);

        encoder = new CANCoder(encoderID);

        steerPID = new PID(steerPIDValue[0], steerPIDValue[1], steerPIDValue[2]);
        drivePID = new PID(drivePIDValue[0], drivePIDValue[1], drivePIDValue[2]);

        x = X;
        y = Y;

        encoderOffs = ENCOFFS;

        this.calcPosition(0, 0);
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
        //why not use mod?
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

    public void resetWheel() {
        steerMotor.set(steerPID.calc(getSwerveHeadingError()));
    }

    public void drive() {
        steerMotor.set(steerPID.calc(getSwerveHeadingError()));

        driveMotor.set(drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity()));
    }

    public void reset() {
        steerPID.reset();
        drivePID.reset();
        steerMotor.set(0);
        driveMotor.set(0);
    }

}
