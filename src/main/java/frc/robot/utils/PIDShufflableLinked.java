package frc.robot.utils;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDShufflableLinked extends PID {
    public String name;

    private PersistentShufflableDouble pUpdater;
    private PersistentShufflableDouble iUpdater;
    private PersistentShufflableDouble dUpdater;

    public PIDShufflableLinked(PersistentShufflableDouble P, PersistentShufflableDouble I,
            PersistentShufflableDouble D, String PIDName) {
        super(P.get(), I.get(), D.get());
        pUpdater = P;
        iUpdater = I;
        dUpdater = D;

        super.lastTime = System.currentTimeMillis();

        name = PIDName;
    }

    public void shuffleUpdatePID() {
        super.p = pUpdater.subscribeAndSet();
        super.i = iUpdater.subscribeAndSet();
        super.d = dUpdater.subscribeAndSet();
        super.reset();
    }

    public boolean detectChange() {
        return pUpdater.detectChanges() || iUpdater.detectChanges() || dUpdater.detectChanges();
    }
}
