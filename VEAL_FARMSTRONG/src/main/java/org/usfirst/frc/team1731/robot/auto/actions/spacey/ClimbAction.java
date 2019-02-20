package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.Climber;

/**
 * Controls Climbers, can take an Climber.WantedState
 */
public class ClimbAction implements Action {

    private Climber mClimber = Climber.getInstance();
    private double mStartTime;
    private Climber.WantedState myDesiredState;

    public ClimbAction() {
        myDesiredState = Climber.WantedState.EXTENDING;
    }

    public ClimbAction(Climber.WantedState desiredState) {
        myDesiredState = desiredState;
        
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp()-mStartTime>=2;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        mClimber.setWantedState(Climber.WantedState.IDLE);

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mClimber.setWantedState(myDesiredState);
        
    }
}
