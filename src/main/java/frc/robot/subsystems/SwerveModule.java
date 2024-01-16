package frc.robot.subsystems;

// import frc.robot.utils.PIDShufflable;
import static frc.robot.utils.mapping.mapValue;

//import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import frc.robot.Constants.DriveConstants;
import frc.robot.utils.PID;

public class SwerveModule {

    public final CANSparkMax steerMotor;
    public final CANSparkMax driveMotor;
    public final CANcoder encoder;
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

    public int swerveModuleID;
    public static int swerveModuleCount = 0;
    public final ShuffleboardTab swerveTab;
    public final GenericEntry steerMotorCurrentWidget;
    public final GenericEntry driveMotorCurrentWidget;
    public final GenericEntry driveMotorSetWidget;
    public final GenericEntry posiitonWidget;
    public final GenericEntry tempWidget;

    public SwerveModule(int STEERMOTOR, int DRIVEMOTOR, double driveConversion, double[] steerPIDValue,
            double[] drivePIDValue, int encoderID, double X, double Y, double ENCOFFS, String SwerveModuleName, SensorDirectionValue directionOnEncoder) {
        swerveModuleID = swerveModuleCount;

        steerPID = new PID(steerPIDValue[0], steerPIDValue[1], steerPIDValue[2]);
        drivePID = new PID(drivePIDValue[0], drivePIDValue[1], drivePIDValue[2]);
        steerMotor = new CANSparkMax(STEERMOTOR, MotorType.kBrushless);
        driveMotor = new CANSparkMax(DRIVEMOTOR, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(driveConversion);
        driveMotor.getEncoder().setVelocityConversionFactor(driveConversion);
        encoder = new CANcoder(encoderID);
        //encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(directionOnEncoder));

        x = X;
        y = Y;
        encoderOffs = ENCOFFS;

        this.calcPosition(0, 0);

        swerveTab = Shuffleboard.getTab("SwerveModules");
        tempWidget = swerveTab.add("Target " + SwerveModuleName, 0).withPosition(9, swerveModuleID)
                .withSize(1, 1)
                .getEntry();
        steerMotorCurrentWidget = swerveTab.add("steerMotor" + SwerveModuleName, 0).withPosition(5, swerveModuleID)
                .withSize(1, 1)
                .getEntry();
        driveMotorCurrentWidget = swerveTab.add("drive motor" + SwerveModuleName, 0).withPosition(6,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        posiitonWidget = swerveTab.add("orientation" + SwerveModuleName, 0).withPosition(7,
                swerveModuleID).withSize(1, 1)
                .getEntry();
        driveMotorSetWidget = swerveTab.add("dmSET" + SwerveModuleName, 0).withPosition(8, swerveModuleID).withSize(1, 1)
                .getEntry();
        swerveModuleCount++;

        // updateShufflables();
    }

    public double getEncoderPosition() {
        return encoder.getAbsolutePosition().getValue()*360 - encoderOffs;
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

       // double val = steerPID.calc(getSwerveHeadingError());
       double val = steerPID.calc(getSwerveHeadingError());
        updateWidget2(val);

        steerMotor.set(val);

        driveMotor.set(mapValue(getSwerveHeadingError(), 0, 180, 1, 0)
                * drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity()));
    }

    public void reset() {
        steerPID.reset();
        drivePID.reset();
        steerMotor.set(0);
        driveMotor.set(0);
    }

    public void updateWidget2(double val)
    {
        steerMotorCurrentWidget.setDouble(val);
    }

    public void updateWidget() {
        //steerMotorCurrentWidget.setDouble(steerMotor.getOutputCurrent());
        
        driveMotorCurrentWidget.setDouble(driveMotor.getOutputCurrent());
        tempWidget.setDouble(getSwerveHeadingError());
        driveMotorSetWidget.setDouble(mapValue(Math.abs(getSwerveHeadingError()), 0, 180, 1, 0)
                * (drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity())));
                
        posiitonWidget.setDouble(getEncoderPosition());
    }

    // public void updateShufflables() {
    //     // if (steerPID.detectChange()) {
    //     //     steerPID.shuffleUpdatePID();
    //     // }
    //     // if (drivePID.detectChange()) {
    //     //     drivePID.shuffleUpdatePID();
    //     // }
    // }

}
