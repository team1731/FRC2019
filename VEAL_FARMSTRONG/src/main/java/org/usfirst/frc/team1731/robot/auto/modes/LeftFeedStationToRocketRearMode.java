package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
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
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamEjectHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamParallelPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TurnToHeadingAction;
import org.usfirst.frc.team1731.robot.auto.actions.WaitAction;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_A;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketRearPath1;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketRearPath2;
import org.usfirst.frc.team1731.robot.paths.LeftRocketRearToFeedStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftRocketRearToFeedStationPath2;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.paths.spacey.Path_1_B;

/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class LeftFeedStationToRocketRearMode extends MirrorableMode {

    @Override
    protected void routine() throws AutoModeEndedException {
    	System.out.println("Executing LeftFeedStationToRocketRearMode");
    	
    	PathContainer Path = new LeftFeedStationToRocketRearPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(150.0))));
        runAction(new TractorBeamEjectHatchAction());
        runAction(new WaitAction(0.5));
        Path = new LeftRocketRearToFeedStationPath1();
        runAction(new ResetPoseFromPathAction(Path));
        runAction(new DrivePathAction(Path));
        Path = new LeftRocketRearToFeedStationPath2();
        runAction(new ParallelAction(Arrays.asList(new Action[] {
            new TractorBeamParallelPickupHatchAction(), 
            new DrivePathAction(Path)
        }))); 
      //  runAction(new DrivePathAction(Path));
       // runAction(new TractorBeamPickupHatchAction());


    }
}
