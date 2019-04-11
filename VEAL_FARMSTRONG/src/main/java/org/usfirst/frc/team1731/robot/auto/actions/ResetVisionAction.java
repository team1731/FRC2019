package org.usfirst.frc.team1731.robot.auto.actions;

import java.util.Optional;

import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;

/**
 * Action to begin shooting.
 * 
 * @see Action
 * @see RunOnceAction
 */
public class ResetVisionAction extends RunOnceAction implements Action {

    @Override
    public void runOnce() {
        
        Optional<ShooterAimingParameters> AimParams;

        RobotState.getInstance().resetVision();
        AimParams = RobotState.getInstance().getAimingParameters();
        Drive.getInstance().setAimingParams(AimParams);
     //   Superstructure.getInstance().setWantedState(Superstructure.WantedState.SHOOT);
  //      Intake.getInstance().setOn(); // maybe intake a few missed balls if we're lucky
    }

}
