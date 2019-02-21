package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.Elevator;
import org.usfirst.frc.team1731.robot.Constants;

//import edu.wpi.first.wpilibj.Timer;

/**
 * Action Interface, an interface that describes an iterative action. It is run by an autonomous action, called by the
 * method runAction in AutoModeBase (or more commonly in autonomous modes that extend AutoModeBase)
 *
  * @see com.team254.frc2017.auto.AutoModeBase#runAction
 */
public class ElevatorAction implements Action {
    //TODO: Check elevator positions we want

    private Elevator mElevator = Elevator.getInstance();
    private double myDesiredPosition;
    public enum DesiredPos {
        HOME,
        BALL_PICKUP,
        BALL_FLOOR,
        BALL_SHIP,
        BALL_2,
        BALL_3,
        HATCH_FLOOR,
        HATCH_SHIP,
        HATCH_2,
        HATCH_3
    }

    private double PositionToDouble(DesiredPos position){
        double returnValue = Constants.kElevatorHomeEncoderValue;

        switch(position){
            case HOME:
                //returnValue defaults to kElevatorHomeEncoderValue
            case BALL_PICKUP:
                returnValue = Constants.kElevatorBallPickup_EncoderValue;
                break;
            case BALL_FLOOR:
                returnValue = Constants.kElevatorBallPickup_EncoderValue;
                break;
            case BALL_SHIP:
                returnValue = Constants.kElevatorCargoShip_EncoderValue;
                break;
            case BALL_2:
                returnValue = Constants.kElevatorCargo2nd_EncoderValue;
                break;
            case BALL_3:
                returnValue = Constants.kElevatorCargo3rd_EncoderValue;
                break;
            case HATCH_FLOOR:
                returnValue = Constants.kElevatorHatchFloor_EncoderValue;
                break;
            case HATCH_SHIP:
                returnValue = Constants.kElevatorHatchShip_EncoderValue;
                break;
            case HATCH_2:
                returnValue = Constants.kElevatorHatch2nd_EncoderValue;
                break;
            case HATCH_3:
                returnValue = Constants.kElevatorHatch2nd_EncoderValue;
                break;
        }

        return returnValue;
    }

    //private double startTime;
    public ElevatorAction(){
        myDesiredPosition = Constants.kElevatorHomeEncoderValue;
    }

    public ElevatorAction(DesiredPos desiredPosition){
        myDesiredPosition = PositionToDouble(desiredPosition);
    }

    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     * 
     * @return boolean
     */
    @Override
    public boolean isFinished(){
        return mElevator.atDesired();
    };

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic lives in this
     * method
     */
    @Override
    public void update(){
    };

    /**
     * Run code once when the action finishes, usually for clean up
     */
    @Override
    public void done(){
        mElevator.setWantedState(Elevator.WantedState.IDLE);
    };

    /**
     * Run code once when the action is started, for set up
     */
    @Override
    public void start(){
        //startTime = Timer.getFPGATimestamp();
        mElevator.setWantedPosition(myDesiredPosition);
        mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
    };
}
