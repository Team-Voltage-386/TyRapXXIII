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

    /**
     * @param STEERMOTOR CAN ID of the steering motor
     * @param DRIVEMOTOR CAN ID of the drive motor
     * @param driveConversion encoder conversion factor
     * @param steerPIDValue steering [P, I, D] as a double array
     * @param drivePIDValue driving [P, I, D] as a double array
     * @param encoderID CAN ID of the encoder
     * @param X X position of the module relative to robot center (forward is positive)
     * @param Y Y position of the module relative to robot center (right positive) (TBD may be left positive)
     * @param ENCOFFS absolute encoder offset (for centering modules)
     */

    public SwerveModule(int STEERMOTOR, int DRIVEMOTOR, double driveConversion, double[] steerPIDValue,
            double[] drivePIDValue, int encoderID, double X, double Y, double ENCOFFS) {
        // Create motors and set conversion factors
        steerMotor = new CANSparkMax(STEERMOTOR, MotorType.kBrushless);
        driveMotor = new CANSparkMax(DRIVEMOTOR, MotorType.kBrushless);
        driveMotor.getEncoder().setPositionConversionFactor(driveConversion);
        driveMotor.getEncoder().setVelocityConversionFactor(driveConversion);
        // This is the steering encoder
        encoder = new CANCoder(encoderID);

        // Set up the PID, values are in constants
        steerPID = new PID(steerPIDValue[0], steerPIDValue[1], steerPIDValue[2]);
        drivePID = new PID(drivePIDValue[0], drivePIDValue[1], drivePIDValue[2]);

        // Set the module position values and encoder offsets
        x = X;
        y = Y;
        encoderOffs = ENCOFFS;

        //Calculates the position and angle of the swerve module relative to the center of the robot
        this.calcPosition(0, 0);
    }

    /**
     * @return the angle of the swerve drive
     */
    public double getEncoderPosition() {
        return encoder.getAbsolutePosition() - encoderOffs;
    }

    /**
     * Calculates the distance from the center and the angle from the center of the swerve module based on the x and y offsets
     * @param offX
     * @param offY
     */
    public void calcPosition(double offX, double offY) {
        distFromCenter = Math.sqrt(Math.pow(x + offX, 2) + Math.pow(y + offY, 2));
        angleFromCenter = Math.toDegrees(Math.atan2(y + offY, x + offX));
    }

    /**Gets the error ranging from -90 to 90 that the swerve drive needs to turn from
     * <p>
     * also sets the drive multiplier to account for when it is quicker to drive the motor backwards rather than steering it all the way around
     */
    private double getSwerveHeadingError() {
        double res = targetSteer - getEncoderPosition();
        while (res < -180)
            res += 360;
            // makes sure that error is in the range of -180 to 180
        while (res > 180)
            res -= 360;
            // makes sure that error is in the range of -180 to 180
        // if it is quicker to drive motor backwards, set drive multiplier and adjust error
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

    /** Calculates and sets motor drive powers */
    public void drive() {
        steerMotor.set(steerPID.calc(getSwerveHeadingError()));

        driveMotor.set(drivePID.calc((driveMult * targetDrive) - driveMotor.getEncoder().getVelocity()));
    }

    /** Resets the control PIDs and sets drive motors to zero */
    public void reset() {
        steerPID.reset();
        drivePID.reset();
        steerMotor.set(0);
        driveMotor.set(0);
    }

}
