package frc.robot.utils;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericEntry;
public class PersistentShufflableDouble {
    public ShuffleboardTab sdtab;
    private double value;
    private GenericEntry ntEntry;
    public PersistentShufflableDouble(double initialValue, String tabName, String entryName){
        sdtab=Shuffleboard.getTab(tabName);
        value=initialValue;
        sdtab.addPersistent(entryName, value).getEntry();
    }
    public double get(){return value;}
    public void set(double Value){value=Value;}

    public double subscribe(){return ntEntry.getDouble(value);}
    public void publish(double Value){ntEntry.setDouble(Value);}
    public double subscribeAndSet(){set(ntEntry.getDouble(value));return value;}
    public void publishAndSet(double Value){ntEntry.setDouble(Value);set(Value);}
}
