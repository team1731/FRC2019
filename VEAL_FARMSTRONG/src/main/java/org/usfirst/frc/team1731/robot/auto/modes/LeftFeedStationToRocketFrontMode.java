package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ParallelAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamEjectHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TurnToHeadingAction;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketFrontPath1;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketFrontPath2;
import org.usfirst.frc.team1731.robot.paths.LeftRocketFrontToFeedStationPath1;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.robot.paths.PathContainer;


/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class LeftFeedStationToRocketFrontMode extends  MirrorableMode {

    @Override
    protected void routine() throws AutoModeEndedException {
    	System.out.println("Executing LeftFeedStationToRocketFrontMode");
        
        PathContainer Path = new LeftFeedStationToRocketFrontPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(30.0))));
        runAction(new TractorBeamEjectHatchAction());
        Path = new LeftRocketFrontToFeedStationPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180.0)));
        runAction(new TractorBeamPickupHatchAction());

    }
}
