package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

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
import org.usfirst.frc.team1731.robot.auto.actions.SpitAction;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_A;
import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.paths.RightRocketRearToFeedStationPath;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_B;

/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class RightRocketRearToFeedStationMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
    	System.out.println("Executing RightRocketRearToFeedStationMode");
    	
    	PathContainer Path = new RightRocketRearToFeedStationPath();
    	runAction(new ResetPoseFromPathAction(Path));
        runAction(new ParallelAction(Arrays.asList(new Action[] {
        		new DrivePathAction(Path)
        })));
    }
}
