package org.usfirst.frc.team1731.robot.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ParallelAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetVisionAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamEjectHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamParallelPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TractorBeamPickupHatchAction;
import org.usfirst.frc.team1731.robot.auto.actions.TurnToHeadingAction;
import org.usfirst.frc.team1731.robot.auto.actions.WaitAction;
import org.usfirst.frc.team1731.robot.paths.LeftCargo1ToFeederStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftCargo1ToFeederStationPath2;
import org.usfirst.frc.team1731.robot.paths.LeftCargo2ToFeederStationPath1;
import org.usfirst.frc.team1731.robot.paths.LeftCargo2ToFeederStationPath2;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToCargoShipH2;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketFrontPath1;
import org.usfirst.frc.team1731.robot.paths.LeftFeedStationToRocketFrontPath2;
import org.usfirst.frc.team1731.robot.paths.LeftLevel1ToCargoL1Path1;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import org.usfirst.frc.team1731.robot.subsystems.Drive;


/**
 * Scores the preload gear onto the boiler-side peg then deploys the hopper and shoots all 60 balls (10 preload + 50
 * hopper).
 * 
 * This was the primary autonomous mode used at SVR, St. Louis Champs, and FOC.
 * 
 * @see AutoModeBase
 */
public class LeftLevel1ToCargoL1Mode extends MirrorableMode {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Executing LeftLevel1ToCargoL1Mode");
       
    	Drive.getInstance().setGyroAngle( Rotation2d.fromDegrees(180.0));
    	PathContainer Path = new LeftLevel1ToCargoL1Path1();
        runAction(new ResetPoseFromPathAction(Path));  
        runAction(new DrivePathAction(Path));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(-90.0))));

        runAction(new TractorBeamEjectHatchAction());
        runAction(new WaitAction(0.5));
        Path = new LeftCargo1ToFeederStationPath1();
       // runAction(new ResetPoseFromPathAction(Path));  //location is probably pretty good given this happens once
        runAction(new DrivePathAction(Path));
      //  runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(-90.0)));
        Path = new LeftCargo1ToFeederStationPath2();
      //  runAction(new DrivePathAction(Path));

        runAction(new ParallelAction(Arrays.asList(new Action[] {
            new TractorBeamParallelPickupHatchAction(), 
            new DrivePathAction(Path)
        })));

      //  runAction(new TractorBeamPickupHatchAction());
        runAction(new WaitAction(0.5));

      //  runAction(new ParallelAction(Arrays.asList(new Action[] {
      //      new TractorBeamPickupHatchAction(), 
      //      new DrivePathAction(Path)
      //  })));
      
      // if you want to go to the rocket use the next 5 lines and comment out the chunk below
     // Path = new LeftFeedStationToRocketFrontPath1();
     // runAction(new ResetPoseFromPathAction(Path));
     // runAction(new DrivePathAction(Path));
    //  runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(30.0))));
     // runAction(new TractorBeamEjectHatchAction());


      // if you want to go back to the cargo ship do these:
         Path = new LeftFeedStationToCargoShipH2();
         runAction(new ResetPoseFromPathAction(Path));
         runAction(new DrivePathAction(Path));
         runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(getAngle(-90.0))));
         runAction(new ResetVisionAction());
         runAction(new TractorBeamEjectHatchAction());
         runAction(new WaitAction(0.5));

         Path = new LeftCargo2ToFeederStationPath1();
         // runAction(new ResetPoseFromPathAction(Path));  //location is probably pretty good given this happens once
          runAction(new DrivePathAction(Path));
        //  runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(-90.0)));
          Path = new LeftCargo2ToFeederStationPath2();
        //  runAction(new DrivePathAction(Path));
  
          runAction(new ParallelAction(Arrays.asList(new Action[] {
              new TractorBeamParallelPickupHatchAction(), 
              new DrivePathAction(Path)
          })));
  
        //  runAction(new TractorBeamPickupHatchAction());
          runAction(new WaitAction(0.2));
 

    }
}

