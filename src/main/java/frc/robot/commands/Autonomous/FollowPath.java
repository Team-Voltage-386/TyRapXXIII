package frc.robot.commands.Autonomous;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowPath extends CommandBase{
    
    String pathGroupName;
    double[] maxVelocity;
    double[] maxAccel;

    PathPlannerTrajectory PPtraj;
    PathPlannerState PPstate;
    ArrayList<PathPlannerTrajectory> pathGroup;

    public FollowPath(String pathGroupName, double[] maxVelocity, double[] maxAccel) {
        this.pathGroupName = pathGroupName;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;

        PPstate = (PathPlannerState)PPtraj.sample(0);
        pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathGroupName, 
            new PathConstraints(4, 3), 
            new PathConstraints(2, 2), 
            new PathConstraints(3, 3)
            );
    }
}
