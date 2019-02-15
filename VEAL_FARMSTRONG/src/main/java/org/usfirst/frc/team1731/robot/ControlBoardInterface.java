package org.usfirst.frc.team1731.robot;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {

		// DRIVER CONTROLS

		//boolean getElevatorButton();

		boolean getFlipUpButton();

		boolean getFlipDownButton();

		double getElevatorControl();

		double getThrottle();

		double getTurn();

		boolean getQuickTurn();

		boolean getLowGear();

		boolean getBlinkLEDButton();

		boolean getGrabCubeButton();

		boolean getOverTheTopButton();

		boolean getCalibrateUp();

		boolean getCalibrateDown();

		boolean getSpit();

		int getClimber();

		boolean getAutoPickUp();

		//DRIVER
		boolean getFrontCamera();
		boolean getBackCamera();
		boolean getActivateAuto();
		boolean getDeactivateAuto();
		boolean getInvertDrive();
		boolean getTestWrist();

		//OPERATOR
		//boolean getFloorLevel();
		//boolean getSecondLevel();
		//boolean getThirdLevel();
		boolean getPickupPanel();
		boolean getShootPanel();
		boolean getPickupBall();
		boolean getShootBall();
		boolean getCargoShipBall();
		boolean getStartingConfiguration();
}
