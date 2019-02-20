package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.Wrist;
import org.usfirst.frc.team1731.robot.subsystems.Wrist.WristPositions;
import org.usfirst.frc.team1731.robot.subsystems.Wrist.WantedState;;

//import edu.wpi.first.wpilibj.Timer;

/**
 * Action Interface, an interface that describes an iterative action. It is run by an autonomous action, called by the
 * method runAction in AutoModeBase (or more commonly in autonomous modes that extend AutoModeBase)
 *
  * @see com.team254.frc2017.auto.AutoModeBase#runAction
 */
public class WristAction implements Action {

    private Wrist mWrist = Wrist.getInstance();
    private WristPositions myDesiredPosition;
    //private double startTime;
    public WristAction(){
        myDesiredPosition = WristPositions.STARTINGPOSITION;
    }

    public WristAction(Wrist.WristPositions desiredPosition){
        myDesiredPosition = desiredPosition;
    }

    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     * 
     * @return boolean
     */
    @Override
    public boolean isFinished(){
        return mWrist.atDesired();
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
        mWrist.setWantedState(WantedState.IDLE);
    };

    /**
     * Run code once when the action is started, for set up
     */
    @Override
    public void start(){
        //startTime = Timer.getFPGATimestamp();
        mWrist.setWantedPosition(myDesiredPosition);
        mWrist.setWantedState(WantedState.WRISTTRACKING);
    };
}
