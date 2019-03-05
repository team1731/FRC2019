package org.usfirst.frc.team1731.robot.subsystems;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

//import com.ctre.PigeonImu.StatusFrameRate;

// com.ctre.PigeonImu.StatusFrameRate;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.ParamEnum;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * 1731 this system controls the wrist
 * 
 * @see Subsystem.java
 */

@SuppressWarnings("unused")
public class Wrist extends Subsystem {

    private static Wrist sInstance = null;
    
    public static Wrist getInstance() {
        if (sInstance == null) {
            sInstance = new Wrist();
        }
        return sInstance;
    }

    //private final TalonSRX mTalon;
    
    public Wrist() {
        // mTalon = new TalonSRX(Constants.kWristTalon);
        // mTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        // mTalon.set(ControlMode.Position, 0);
        // mTalon.selectProfileSlot(Constants.SlotIdx, 0);
        // mTalon.config_kP(Constants.SlotIdx, Constants.kWristTalonKP, Constants.kTimeoutMs );
        // mTalon.config_kI(Constants.SlotIdx, Constants.kWristKI, Constants.kTimeoutMs );
        // mTalon.config_kD(Constants.SlotIdx, Constants.kWristKD, Constants.kTimeoutMs);
        // mTalon.config_kF(Constants.SlotIdx, Constants.kWristTalonKF, Constants.kTimeoutMs );
        // mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        // mTalon.configClosedloopRamp(0, Constants.kTimeoutMs);
        // mTalon.overrideLimitSwitchesEnable(false);
        
        // mTalon.setNeutralMode(NeutralMode.Brake);

        // /* choose to ensure sensor is positive when output is positive */
        // mTalon.setSensorPhase(Constants.kSensorPhase);

        // /* choose based on what direction you want forward/positive to be.
        //  * This does not affect sensor phase. */ 
        // mTalon.setInverted(true); //Constants.kMotorInvert);

        // /* set the peak and nominal outputs, 12V means full */
        // mTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
        // mTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        // mTalon.configPeakOutputForward(0.75, Constants.kTimeoutMs);
        // mTalon.configPeakOutputReverse(-0.5, Constants.kTimeoutMs);
        // /*
        //  * set the allowable closed-loop error, Closed-Loop output will be
        //  * neutral within this range. See Table in Section 17.2.1 for native
        //  * units per rotation.
        //  */
        // mTalon.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);

    }
    	
    public enum SystemState {	
        IDLE,   // stop all motors
        WRISTTRACKING // moving
    }

    public enum WantedState {
    	IDLE,   
        WRISTTRACKING // moving
    }

    public enum WristPositions {
    	CARGOPICKUP(522),   
        STRAIGHTAHEAD(580), // moving
        SHOOTHIGH(588),
        STARTINGPOSITION(588);

        private final int units;

        private WristPositions(int units){
            this.units = units;
        }

        private int getPos(){
            return units;
        }
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private WristPositions mCurrentPosition;
    private WristPositions mWantedPosition = WristPositions.STARTINGPOSITION;
    private boolean mPositionChanged = false;
    private double mNextEncPos = 0;
    private boolean mStateChanged = false;
    private boolean mRevSwitchSet = false;



    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Wrist.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentPosition = WristPositions.STARTINGPOSITION;
                mWantedPosition = WristPositions.STARTINGPOSITION;
                mPositionChanged = false;
                mCurrentStateStartTime = timestamp;
                //mTalon.setSelectedSensorPosition(0, 0, 10);             
              //  DriverStation.reportError("Wrist SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
   	
        	synchronized (Wrist.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case WRISTTRACKING:
                        newState = handleWristTracking();
                        break;
                    default:
                        newState = SystemState.IDLE;                    
                }

                if (newState != mSystemState) {
                    System.out.println("Wrist state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    //DriverStation.reportWarning("Wrist SystemState: " + mSystemState, false);
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

		@Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case WRISTTRACKING:
                return SystemState.WRISTTRACKING;
            default:
                return SystemState.IDLE;
        }
    }
    
    private SystemState handleIdle() {
        if (mStateChanged) {
            //mTalon.set(ControlMode.PercentOutput, 0);
        }
        return defaultStateTransfer();
    }

    private SystemState handleWristTracking() {
        if (mPositionChanged) {
            //mTalon.set(ControlMode.Position, mWantedPosition.getPos());
            mPositionChanged = false;
        }
	    return defaultStateTransfer();
    }

    public synchronized void setWantedPosition(WristPositions position) {
        mWantedPosition = position;
        mPositionChanged = true;
        setWantedState(WantedState.WRISTTRACKING);
    }

    public synchronized int getCurrentPosition(boolean up) {
        return 0;
    }
    
    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
            //DriverStation.reportError("Wrist WantedState: " + mWantedState, false);
        }
    }

    
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("WristSysState", mSystemState.name()); // .ordinal());
        SmartDashboard.putString("WristWantState", mWantedState.name());
        //SmartDashboard.putNumber("WristWantState", (double)mWantedState.ordinal());
        SmartDashboard.putString("WristWantPos", mWantedPosition.toString());
        //SmartDashboard.putNumber("WristCurPos", mTalon.getSelectedSensorPosition(0));
        //SmartDashboard.putNumber("WristQuadPos", mTalon.getSensorCollection().getQuadraturePosition());
        //SmartDashboard.putBoolean("WristRevSw", mTalon.getSensorCollection().isRevLimitSwitchClosed());
        //SmartDashboard.putBoolean("WristLastRevSw", mRevSwitchSet);
    }

    @Override
    public void stop() {
        zeroSensors();
        setWantedState(WantedState.IDLE);
    }

    public boolean atDesired() {
    	return false; //Math.abs(mTalon.getSelectedSensorPosition(0) - mNextEncPos)<100;
    }
    
    @Override
    public void zeroSensors() {
        //mTalon.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing Wrist.-----------------------------------");
        return false;
    }

}
