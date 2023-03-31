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
    PathConstraints[] pathConstraints;

    //*Follows group of paths for auto */
    public FollowPath(String pathGroupName, double[] maxVelocity, double[] maxAccel) {
        this.pathGroupName = pathGroupName;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;

        //Making array of pathConstraints excluding the first one so var arg pathconstraints in loadPathGroup can accept it
        pathConstraints = new PathConstraints[maxVelocity.length - 1];
        for(int i = 1; i < maxVelocity.length; i++){
            pathConstraints[i] = new PathConstraints(maxVelocity[i], maxAccel[i]);
        }

        //init pathGroup
        if(maxVelocity.length == 2)
        pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            pathGroupName, 
            new PathConstraints(maxVelocity[0], maxAccel[0]), 
            pathConstraints);
    }
}
