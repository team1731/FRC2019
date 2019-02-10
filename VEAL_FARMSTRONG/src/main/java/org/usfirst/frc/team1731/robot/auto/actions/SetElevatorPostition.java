package org.usfirst.frc.team1731.robot.auto.actions;

import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.Constants.ELEVATOR_POSITION;

/**
 * Action to begin shooting.
 * 
 * @see Action
 * @see RunOnceAction
 */
public class SetElevatorPostition extends RunOnceAction implements Action {

	private boolean up = true;
	

	
    @Override
    public void runOnce() {

        Superstructure.getInstance().setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_FLOOR);
 
    }

}
