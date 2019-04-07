package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ElevatorHome;
import org.usfirst.frc.team1731.robot.auto.actions.ElevatorUp;
import org.usfirst.frc.team1731.robot.auto.actions.ParallelAction;
import org.usfirst.frc.team1731.robot.auto.actions.PickUpAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.auto.actions.RotateIntakeActionUp;
import org.usfirst.frc.team1731.robot.auto.actions.TurnToHeadingAction;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_A;
import org.usfirst.frc.team1731.robot.paths.DynamicPath;
import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath2;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_B;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import java.util.List;

/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class PlacePanel extends AutoModeBase {
    RobotState mRobotState = RobotState.getInstance();
    Waypoint start;
    Waypoint end;
    Double startx;
    Double starty;
    Double endx;
    Double endy;


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Executing PlacePanel");

        RigidTransform2d odometry = mRobotState.getLatestFieldToVehicle().getValue();
        startx = odometry.getTranslation().x();
        starty = odometry.getTranslation().y();
        start = new Waypoint(startx, starty, 0, 20);

        List<RigidTransform2d> poses = mRobotState.getCaptureTimeFieldToGoal();
        for (RigidTransform2d pose : poses) {
            // Only output first goal
            endx = pose.getTranslation().x() + startx;
            endy = pose.getTranslation().y() + starty;
            break;
        }
    	end = new Waypoint(endx, endy, 0, 20);  // need to determine end of path from aiming parameters
    	PathContainer Path = new DynamicPath(start, end);
        runAction(new DrivePathAction(Path));

        //eject panel

        //backup
        odometry = mRobotState.getLatestFieldToVehicle().getValue();
        startx = odometry.getTranslation().x();
        starty = odometry.getTranslation().y();
        start = new Waypoint(startx, starty, 0, 20);
    	end = new Waypoint(startx - 20, starty, 0, 20);  // need to determine end of path from aiming parameters
    	Path = new DynamicPath(start, end);
        runAction(new DrivePathAction(Path));

    }
}
