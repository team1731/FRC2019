package org.usfirst.frc.team1731.robot.auto.modes;

import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.paths.LeftLevel1ToCargoL1Path1;
import org.usfirst.frc.team1731.robot.paths.PathContainer;


/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class LeftLevel1ToCargoL1Mode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
    	System.out.println("Executing LeftLevel1ToCargoL1Mode");
    	
    	PathContainer Path = new LeftLevel1ToCargoL1Path1();
        runAction(new ResetPoseFromPathAction(Path));  
        runAction(new DrivePathAction(Path));
      //  runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0.0)));
      //  Path = new LeftFeedStationToRocketFrontPath2();
      //  runAction(new DrivePathAction(Path));

    }
}
