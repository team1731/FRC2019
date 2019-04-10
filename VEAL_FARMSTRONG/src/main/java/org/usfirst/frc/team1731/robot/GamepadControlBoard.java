package org.usfirst.frc.team1731.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Contains the button mappings for the Gamepad control board.  Like the drive code, one instance of the GamepadControlBoard 
 * object is created upon startup, then other methods request the singleton GamepadControlBoard instance.  Implements the 
 * ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class GamepadControlBoard implements ControlBoardInterface {

    private final Joystick mDriver;
    private final Joystick mOperator;

    private Boolean invertDrivePrevious = Boolean.FALSE;
    private Boolean toggleDriveSpeedPrevious = Boolean.FALSE;
    private static ControlBoardInterface mInstance = null;
    
    public static ControlBoardInterface getInstance() {
    	if (mInstance == null) {
    		mInstance = new GamepadControlBoard();
    	}
    	return mInstance;
    }

    protected GamepadControlBoard() {
        mDriver = new Joystick(0);
    	mOperator = new Joystick(1);
    }
    
    @Override
    public void rumbleDriver(){
        mDriver.setRumble(RumbleType.kLeftRumble, 1.0);
        mDriver.setRumble(RumbleType.kRightRumble, 1.0);
    }

    @Override
    public boolean getGrabCubeButton() {
    	 return false; // Math.abs(mOperator.getRawAxis(3)) > .8;
    }
    
    @Override
    public boolean getTractorDrivePickupHatch() {
        boolean left_trigger = (Math.abs(mDriver.getRawAxis(2)) > 0.8);
        return left_trigger;
    }
    @Override
    public boolean getTractorDriveEjectHatch() {
        boolean right_trigger = (Math.abs(mDriver.getRawAxis(3)) > 0.8);
        return right_trigger;
    }
    
    @Override
    public boolean getSpit() {
        return false; // Math.abs(mOperator.getRawAxis(2)) > .8;
    }
    
    @Override
    public boolean getCalibrateDown() {
        return mOperator.getRawButton(7);
    }
    
    @Override
    public boolean getCalibrateUp() {
        return mOperator.getRawButton(8);
    }


    @Override
    public double getThrottle() {
        return -mDriver.getRawAxis(1);
    }

    boolean getGrabCubeButton = false;  
    
    @Override
    public double getTurn() {
        return mDriver.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn() {
        // R1
        return mDriver.getRawButton(6);
    }

    @Override
    public boolean getLowGear() {
        // L1
        return false; // mDriver.getRawButton(5);

    }

    @Override
    public int getClimber() {
        // A
		//    	Get the angle in degrees of a POV on the HID. 
		//
		//    	The POV angles start at 0 in the up direction, and increase clockwise
    	//		(eg right is 90, upper-left is 315).
		//    	Parameters:pov The index of the POV to read (starting at 0)
    	//		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
		//
        //int pov = mOperator.getPOV(0);    	
        //return ((pov != -1) && (pov > 315 || pov < 45)) &&  mOperator.getRawButton(1);
        if (mDriver.getRawButton(7) && mDriver.getRawButton(8)
                                    && mOperator.getRawButton(2)) { // SAFETY! button B on operator controller)
            return 1; // extend climber/legolift
        }
        return 0; // pause/stop climber
    }
      

    @Override
	public double getElevatorControl() {
        double angle = mOperator.getPOV(0); // getPOV
        
        double result = -1;
        if (angle != -1 && "kXInputGamepad".equalsIgnoreCase(mOperator.getType().toString().trim())) {
            if ((angle > 340) || (angle < 20)) {
                result = 2;
            } else if ((angle > 250) && (angle < 290)) {
                result = 1;
            } else if ((angle > 160) && (angle < 200)) {
                result = 0;
            }
            //} else if ((angle > 70) && (angle < 110)) {
            //    result = 3;
        }
        //System.out.println(mOperator.getType() + "   result=" + result);

        return result;
	}

    @Override
    public boolean getFlipDownButton() {
        return mOperator.getRawButton(3); // getButtonX
    }

    @Override
    public boolean getFlipUpButton() {
        return mOperator.getRawButton(4); // getButtonY
    }

	@Override
    public boolean getAutoPickUp() {
        // R1
        return false; // mOperator.getRawButton(6);
    }
    
    // //DRIVER
	// @Override
    // public boolean getWideCamera(){
    // //
    // //    	Get the angle in degrees of a POV on the HID. 
    // //
    // //    	The POV angles start at 0 in the up direction, and increase clockwise
    // //		(eg right is 90, upper-left is 315).
    // //    	Parameters:pov The index of the POV to read (starting at 0)
    // //		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
    // //  
    //     int pov = mDriver.getPOV(0);
    //     return ((pov !=1) && (pov > 225) && (pov < 315));
    // }

	// @Override
    // public boolean getNormalCamera(){
    // //
    // //    	Get the angle in degrees of a POV on the HID. 
    // //
    // //    	The POV angles start at 0 in the up direction, and increase clockwise
    // //		(eg right is 90, upper-left is 315).
    // //    	Parameters:pov The index of the POV to read (starting at 0)
    // //		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
    // //  
    //     int pov = mDriver.getPOV(0);
    //     return ((pov !=1) && (pov > 45) && (pov < 135));
    // }

      @Override
    public boolean getFrontCamera(){
        int pov = mDriver.getPOV(0);
        return (pov != -1) && (pov > 315 || pov < 45);
    }

	@Override
    public boolean getBackCamera(){
        int pov = mDriver.getPOV(0);
        return ((pov != -1) && (pov > 135) && (pov < 225));
    }

    //////////////////////////
    // DRIVER AUTO CONTROLS
    //////////////////////////
	@Override
    public boolean getAutoRearToFeederStation(){
        return mDriver.getRawButton(1);
    }
    public boolean getAutoFeederStationToRear(){
        return mDriver.getRawButton(4);
    }
    public boolean getAutoFrontToFeederStation(){
        return mDriver.getRawButton(2);
    }
    public boolean getAutoFeederStationToFront(){
        return mDriver.getRawButton(3);
    }

    public boolean getAutoLevel1ToCargoL1(){   
        //return mDriver.getRawButton(7);
        int pov = mDriver.getPOV(0);
        return ((pov !=1) && (pov > 45) && (pov < 135) && "kXInputGamepad".equalsIgnoreCase(mDriver.getType().toString().trim()));
    }
    public boolean getAutoCargoL1ToFeederStation(){
        //return mDriver.getRawButton(8);
        int pov = mDriver.getPOV(0);
        return ((pov !=1) && (pov > 225) && (pov < 315) && "kXInputGamepad".equalsIgnoreCase(mDriver.getType().toString().trim()));
    }
    public boolean getAutoLevel1ToRear(){
        //return mOperator.getRawButton(4);
        int pov = mDriver.getPOV(0);
        return ((pov != -1) && (pov > 315 || pov < 45) && "kXInputGamepad".equalsIgnoreCase(mDriver.getType().toString().trim()));
    }


	@Override
    public boolean getInvertDrive(){
        boolean invertDrive=false;
        synchronized(invertDrivePrevious){
            boolean invertDriveCurrent= mDriver.getRawButton(10);
            if(invertDriveCurrent && !invertDrivePrevious){
                invertDrive=true;
            }
            invertDrivePrevious = invertDriveCurrent;
        }
        return invertDrive;
    }

	@Override
    public boolean getToggleDriveSpeed(){
        boolean toggleDriveSpeed=false;
        synchronized(toggleDriveSpeedPrevious){
            boolean toggleDriveSpeedCurrent= mDriver.getRawButton(9);
            if(toggleDriveSpeedCurrent && !toggleDriveSpeedPrevious){
                toggleDriveSpeed=true;
            }
            toggleDriveSpeedPrevious = toggleDriveSpeedCurrent;
        }
        return toggleDriveSpeed;
    }                                         

    /*
    //OPERATOR
	@Override
    public boolean getFloorLevel(){
    //
    //    	Get the angle in degrees of a POV on the HID. 
    //
    //    	The POV angles start at 0 in the up direction, and increase clockwise
    //		(eg right is 90, upper-left is 315).
    //    	Parameters:pov The index of the POV to read (starting at 0)
    //		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
    //  
        int pov = mOperator.getPOV(0);
        return ((pov != -1) && (pov > 135) && (pov < 225));
    }

	@Override
    public boolean getSecondLevel(){
    //
    //    	Get the angle in degrees of a POV on the HID. 
    //
    //    	The POV angles start at 0 in the up direction, and increase clockwise
    //		(eg right is 90, upper-left is 315).
    //    	Parameters:pov The index of the POV to read (starting at 0)
    //		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
    //  
        int pov = mOperator.getPOV(0);
        return ((pov != -1) && (pov > 225) && (pov < 315));
    }

	@Override
    public boolean getThirdLevel(){
    //
    //    	Get the angle in degrees of a POV on the HID. 
    //
    //    	The POV angles start at 0 in the up direction, and increase clockwise
    //		(eg right is 90, upper-left is 315).
    //    	Parameters:pov The index of the POV to read (starting at 0)
    //		Returns:the angle of the POV in degrees, or -1 if the POV is not pressed.
    //  
        int pov = mOperator.getPOV(0);
        return ((pov != -1) && (pov > 315 || pov < 45));
    }
    */

	@Override
    public boolean getPickupPanel(){
        return Math.abs(mOperator.getRawAxis(2)) > .8;
    }

	@Override
    public boolean getShootPanel(){
        return mOperator.getRawButton(5);
    }

	@Override
    public boolean getPickupBall(){
        return Math.abs(mOperator.getRawAxis(3)) > .8;

    }

	@Override
    public boolean getShootBall(){
        return mOperator.getRawButton(6);
    }

	@Override
    public boolean getCargoShipBall(){
        // ball sensor combined with level button (floor, 2nd level, 3d level) will determine actual elevator position
        return mOperator.getRawButton(3);
    }

	@Override
    public boolean getStartingConfiguration(){
        // ball sensor combined with level button (floor, 2nd level, 3d level) will determine actual elevator position
        return mOperator.getRawButton(1);
    }

}
