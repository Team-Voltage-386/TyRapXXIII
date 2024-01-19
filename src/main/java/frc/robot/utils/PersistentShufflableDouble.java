package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericEntry;

public class PersistentShufflableDouble {
    public ShuffleboardTab sdtab;
    private double value;
    private GenericEntry ntEntry;

    public PersistentShufflableDouble(double initialValue, String entryName) {
        sdtab = Shuffleboard.getTab("persistentshufflabledoubles");
        value = initialValue;
        ntEntry = sdtab.addPersistent(entryName, value).getEntry();
    }

    public PersistentShufflableDouble(double initialValue, String entryName, String TabName) {
        sdtab = Shuffleboard.getTab(TabName);
        value = initialValue;
        ntEntry = sdtab.addPersistent(entryName, value).getEntry();

    }

    public double get() {
        return value;
    }

    /** write the value without shuffleboard */
    public void set(double Value) {
        value = Value;
    }

    /** @deprecated @return pull the double from the shufleboard */
    public double subscribe() {
        return ntEntry.getDouble(value);
    }

    /** @deprecated push a double to shuffleboard */
    public void publish(double Value) {
        ntEntry.setDouble(Value);
    }

    /** @return pull the double from the shuffleboard and write to value */
    public double subscribeAndSet() {
        set(ntEntry.getDouble(value));
        return value;
    }

    /** push the double from the shuffleboard and write to value */
    public void publishAndSet(double Value) {
        ntEntry.setDouble(Value);
        set(Value);
    }

    /** @return detect changes */
    public boolean detectChanges() {
        return ntEntry.getDouble(value) != value;
    }
}
