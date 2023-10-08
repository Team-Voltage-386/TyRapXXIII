package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class PID extends SubsystemBase
{
    @Override
    public void periodic() 
    {
        //Periodically checks SHuffleBoard to see if the PID values need to be updated
        updatePID();
    }

    //The ShuffleBoard tab that the PID values will be retrieved from
    private ShuffleboardTab pidTab;

    //PID values that are currently programmed into the motor
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    //PID values that are on ShuffleBoard
    private SimpleWidget PValue, IValue, DValue, IZone, FeedForward, MaxOutput, MinOutput;

    //The PID controller for the CanSparkMax motor
    private SparkMaxPIDController SparkPIDcontrols;

    public PID(String SuffleBoardTabName, CANSparkMax Motor)
    {
        //Creates a tab in ShuffleBoard for the PID values of the specified CanSparkMax motor
        pidTab = Shuffleboard.getTab(SuffleBoardTabName);

        //Saves the PID controller that is built into the motor and its encoder
        SparkPIDcontrols = Motor.getPIDController();

        //Initializes the PID variables
        kP = 1e-10; 
        kI = 0.1;
        kD = 0.0001; 
        kIz = 1; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        //Creates the Widgerts that will be used in the shuffleboard tab
        PValue = pidTab.add("P Value", kP);
        IValue = pidTab.add("I Value", kI);
        DValue = pidTab.add("D Value", kD);
        IZone= pidTab.add("I Zone", kIz);
        FeedForward = pidTab.add("Feed Forward", kFF);
        MaxOutput = pidTab.add("Max Output", kMaxOutput);
        MinOutput = pidTab.add("Min Output", kMinOutput);

        //Burns the initialized PID values into the CanSparkMax motor
        SparkPIDcontrols.setP(kP);
        SparkPIDcontrols.setI(kI);
        SparkPIDcontrols.setD(kD);
        SparkPIDcontrols.setIZone(kIz);
        SparkPIDcontrols.setFF(kFF);
        SparkPIDcontrols.setOutputRange(kMinOutput, kMaxOutput);
    }

    private void updatePID()
    {
        //Retrieves the values from ShuffleBoard
        double p = PValue.getEntry().getDouble(0);
        double i = IValue.getEntry().getDouble(0);
        double d = DValue.getEntry().getDouble(0);
        double iz = IZone.getEntry().getDouble(0);
        double ff = FeedForward.getEntry().getDouble(0);
        double max = MaxOutput.getEntry().getDouble(0);
        double min = MinOutput.getEntry().getDouble(0);

        //Checks for changes in the P value
        if (kP != p)
        {
            kP = p;
            SparkPIDcontrols.setP(kP);
        }

        //Checks for changes in the I value
        if (kI != i)
        {
            kI = i;
            SparkPIDcontrols.setI(kI);
        }

        //Checks for changes in the D value
        if (kD != d)
        {
            kD = d;
            SparkPIDcontrols.setD(kD);
        }

        //Checks for changes in the I zone value
        if (kIz != iz)
        {
            kIz = iz;
            SparkPIDcontrols.setIZone(kIz);
        }

        //Checks for changes in the feed forward value
        if (kFF != ff)
        {
            kFF = ff;
            SparkPIDcontrols.setFF(kFF);
        }

        //Checks for changes in the maximum input and output values
        if (kMaxOutput != max || kMinOutput != min)
        {
            kMaxOutput = max;
            kMinOutput = min;
            SparkPIDcontrols.setOutputRange(kMinOutput, kMaxOutput);
        }    
    }

    //Controls the motor with PID
    public void setReference(double refrenceValue, ControlType refrenceControlType)
    {
        SparkPIDcontrols.setReference((refrenceValue/360.0*3), refrenceControlType);
    }
}
