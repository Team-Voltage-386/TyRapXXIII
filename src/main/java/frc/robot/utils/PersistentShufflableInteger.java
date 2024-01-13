package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.GenericEntry;

public class PersistentShufflableInteger {
    public ShuffleboardTab sdtab;
    private int value;
    private GenericEntry ntEntry;

    public PersistentShufflableInteger(int initialValue, String entryName) {
        sdtab = Shuffleboard.getTab("persistentshufflableintegers");
        value = initialValue;
        ntEntry = sdtab.addPersistent(entryName, value).getEntry();
    }

    public PersistentShufflableInteger(int initialValue, String entryName, String TabName) {
        sdtab = Shuffleboard.getTab(TabName);
        value = initialValue;
        ntEntry = sdtab.addPersistent(entryName, value).getEntry();

    }

    public int get() {
        return value;
    }

    /** write the value without shuffleboard */
    public void set(int Value) {
        value = Value;
    }

    /** @deprecated @return pull the int from the shufleboard */
    public int subscribe() {
        return (int) ntEntry.getInteger(value);
    }

    /** @deprecated push a int to shuffleboard */
    public void publish(int Value) {
        ntEntry.setInteger(Value);
    }

    /** @return pull the int from the shuffleboard and write to value */
    public int subscribeAndSet() {
        set((int) ntEntry.getInteger(value));
        return value;
    }

    /** push the int from the shuffleboard and write to value */
    public void publishAndSet(int Value) {
        ntEntry.setInteger(Value);
        set(Value);
    }

    /** @return detect changes */
    public boolean detectChanges() {
        return ntEntry.getInteger(value) != value;
    }
}
