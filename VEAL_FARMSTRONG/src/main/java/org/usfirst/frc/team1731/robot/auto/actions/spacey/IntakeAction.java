package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.Intake;

/**
 * Controls LEDs, can take an LED.WantedState
 */
public class IntakeAction implements Action {

    private Intake mIntake = Intake.getInstance();
    private double mStartTime;
    private Intake.WantedState myDesiredState;

    public IntakeAction() {
        myDesiredState = Intake.WantedState.INTAKING;
    }

    public IntakeAction(Intake.WantedState desiredState) {
        myDesiredState = desiredState;
        
    }

    @Override
    public boolean isFinished() {
        return mIntake.hasCargo();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        mIntake.setWantedState(Intake.WantedState.IDLE);

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.setWantedState(myDesiredState);

    }
}
