package org.usfirst.frc.team1731.robot;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {

		// DRIVER CONTROLS

		//boolean getElevatorButton();
		
		void rumbleDriver();

		boolean getFlipUpButton();

		boolean getFlipDownButton();

		double getElevatorControl();

		double getThrottle();

		double getTurn();

		boolean getQuickTurn();

		boolean getLowGear();

		boolean getGrabCubeButton();

		boolean getCalibrateUp();

		boolean getCalibrateDown();

		boolean getSpit();

		int getClimber();

		boolean getAutoPickUp();

		//DRIVER
		boolean getFrontCamera();
		boolean getBackCamera();

		//these all assume LEFT HAND SIDE ROCKET:
		boolean getAutoRearToFeederStation();
		boolean getAutoFeederStationToRear();
		boolean getAutoFrontToFeederStation();
		boolean getAutoFeederStationToFront();
	//	boolean getAutoLevel1ToRear();
		boolean getAutoLevel1ToCargoL1();
		boolean getAutoCargoL1ToFeederStation();
		boolean getAutoLevel1ToRear();


		
		boolean getInvertDrive();
		boolean getToggleDriveSpeed();
		
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

		boolean getTractorDrivePickupHatch();
		boolean getTractorDriveEjectHatch();
}
