package org.usfirst.frc.team1731.robot.auto.actions.spacey;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1731.robot.auto.actions.Action;
import org.usfirst.frc.team1731.robot.subsystems.LED;

/**
 * Controls LEDs, can take an LED.WantedState
 */
public class LEDAction implements Action {

    private LED mLED = LED.getInstance();
    private double mStartTime;
    private LED.WantedState myDesiredState;

    public LEDAction() {
        myDesiredState = LED.WantedState.OFF;
    }

    public LEDAction(LED.WantedState desiredState) {
        myDesiredState = desiredState;
        
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp()-mStartTime>=0.025;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mLED.setWantedState(myDesiredState);
        mLED.setLEDOn();
    }
}
