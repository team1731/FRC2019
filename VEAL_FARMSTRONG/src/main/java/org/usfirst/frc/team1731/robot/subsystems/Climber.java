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

import edu.wpi.first.wpilibj.DigitalInput;


//import com.ctre.phoenix.motorcontrol.StatusFrameRate;
//import com.ctre.phoenix.motorcontrol.VelocityMeasWindow;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * 1731 this system controls the elevator
 * 
 * @see Subsystem.java
 */

//stemrobotics.cs.pdx.edu/sites/default/files/WPILib_programming.pdf

@SuppressWarnings("unused")
public class Climber extends Subsystem {

    private static Climber sInstance = null;
    
    public static Climber getInstance() {
        if (sInstance == null) {
            sInstance = new Climber();
        }
        return sInstance;
    }

    private final TalonSRX mTalonL;
    private final TalonSRX mTalonR;
    
    public Climber() {
        // Left Talon
        mTalonL = new TalonSRX(Constants.kClimberTalonL);
		/* Factory default hardware to prevent unexpected behavior */
		//mTalonL.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
        mTalonL.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		mTalonL.setSensorPhase(Constants.kSensorPhase);
		mTalonL.setInverted(true);  //old was true

		/* Set relevant frame periods to be at least as fast as periodic rate */
		mTalonL.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		mTalonL.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		mTalonL.configNominalOutputForward(0, Constants.kTimeoutMs);
		mTalonL.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mTalonL.configPeakOutputForward(0.8, Constants.kTimeoutMs);
		mTalonL.configPeakOutputReverse(-0.4, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalonL.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalonL.config_kF(Constants.kSlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs);
		mTalonL.config_kP(Constants.kSlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs);
		mTalonL.config_kI(Constants.kSlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs);
		mTalonL.config_kD(Constants.kSlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalonL.configMotionCruiseVelocity(Constants.kClimberCruiseVelocity, Constants.kTimeoutMs);
		mTalonL.configMotionAcceleration(Constants.kClimberAcceleration, Constants.kTimeoutMs);

		/* Zero the sensor */
        //mTalonL.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalonL.set(ControlMode.PercentOutput, 0);

        // FROM WRIST CODE
        //--mTalonL.set(ControlMode.Position, 0);
        //--mTalonL.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        mTalonL.configClosedloopRamp(0, Constants.kTimeoutMs);
        //mTalonL.overrideLimitSwitchesEnable(false);
        mTalonL.setNeutralMode(NeutralMode.Brake);
        /* choose based on what direction you want forward/positive to be.
         * This does not affect sensor phase. */ 
        mTalonL.setInverted(true); //Constants.kMotorInvert);
        /*
         * set the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        mTalonL.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);

        //Right Talon
        mTalonR = new TalonSRX(Constants.kClimberTalonL);
		/* Factory default hardware to prevent unexpected behavior */
		//mTalonR.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
        mTalonR.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		mTalonR.setSensorPhase(Constants.kSensorPhase);
		mTalonR.setInverted(true);  //old was true

		/* Set relevant frame periods to be at least as fast as periodic rate */
		mTalonR.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		mTalonR.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		mTalonR.configNominalOutputForward(0, Constants.kTimeoutMs);
		mTalonR.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mTalonR.configPeakOutputForward(0.8, Constants.kTimeoutMs);
		mTalonR.configPeakOutputReverse(-0.4, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalonR.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalonR.config_kF(Constants.kSlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs);
		mTalonR.config_kP(Constants.kSlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs);
		mTalonR.config_kI(Constants.kSlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs);
		mTalonR.config_kD(Constants.kSlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalonR.configMotionCruiseVelocity(Constants.kClimberCruiseVelocity, Constants.kTimeoutMs);
		mTalonR.configMotionAcceleration(Constants.kClimberAcceleration, Constants.kTimeoutMs);

		/* Zero the sensor */
        //mTalonR.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalonR.set(ControlMode.PercentOutput, 0);

        // FROM WRIST CODE
        //--mTalonR.set(ControlMode.Position, 0);
        //--mTalonR.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        mTalonR.configClosedloopRamp(0, Constants.kTimeoutMs);
        //mTalonR.overrideLimitSwitchesEnable(false);
        mTalonR.setNeutralMode(NeutralMode.Brake);
        /* choose based on what direction you want forward/positive to be.
         * This does not affect sensor phase. */ 
        mTalonR.setInverted(true); //Constants.kMotorInvert);
        /*
         * set the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        mTalonR.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);        
    }

    public enum SystemState {	
        IDLE,   // stop all motors
        EXTENDING, // lego lift extend
        RETRACTING, // lego lift retract
    }

    public enum WantedState {
    	IDLE,   
        EXTENDING, // lego lift extend
        RETRACTING, // lego lift retract
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    //private double mWantedPosition = 0;
    private boolean mStateChanged = false;
    //private boolean mPositionChanged = false;
    //private boolean wasCalibrated = false;
    //private boolean mRevSwitchSet = false;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Climber.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                //mPositionChanged = false;
                //mWantedPosition = 0;
                mCurrentStateStartTime = timestamp;
                //mTalonL.setSelectedSensorPosition(0, 0, 10);                
              //  DriverStation.reportError("Climber SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
   	
        	synchronized (Climber.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case EXTENDING:
                        newState = handleExtending();
                        break;
                    case RETRACTING:
                        newState = handleRetracting();
                        break;
                    default:
                        newState = SystemState.IDLE;                    
                }

                if (newState != mSystemState) {
                    System.out.println("Climber state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    //DriverStation.reportWarning("Climber SystemState: " + mSystemState, false);
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
            case EXTENDING:
                return SystemState.EXTENDING;
            case RETRACTING:
                return SystemState.RETRACTING;

            default:
                return SystemState.IDLE;
        }
    }
    
    private SystemState handleIdle() {
        if (mStateChanged) {
            mTalonL.set(ControlMode.PercentOutput, 0);
            mTalonR.set(ControlMode.PercentOutput, 0);
        }
        return defaultStateTransfer();
    }

    private SystemState handleExtending() {
        if (mStateChanged) {
            //mTalonL.set(ControlMode.PercentOutput, Constants.kClimberExtendPercent);
            mTalonL.set(ControlMode.MotionMagic, Constants.kClimberExtendedPosition);
            mTalonR.set(ControlMode.MotionMagic, Constants.kClimberExtendedPosition);
        }

        return defaultStateTransfer();
    }

    private SystemState handleRetracting() {
        if (mStateChanged) {
            //mTalonL.set(ControlMode.PercentOutput, Constants.kClimberRetractPercent);
            mTalonL.set(ControlMode.MotionMagic, Constants.kClimberRetractedPosition);
            mTalonR.set(ControlMode.MotionMagic, Constants.kClimberRetractedPosition);
        }

        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("ClimbSysState", mSystemState.name());
        SmartDashboard.putString("ClimbWantState", mWantedState.name());
        SmartDashboard.putNumber("ClimbCurPosLeft", mTalonL.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("ClimbCurPosRight", mTalonR.getSelectedSensorPosition(0));
    }

    @Override
    public void stop() {
        // mVictor.set(0);
        setWantedState(WantedState.IDLE);
    }
    
    public synchronized int getCurrentPosition() {
    	return mTalonL.getSelectedSensorPosition(0);
    }
    /*
	public boolean atTop() {
		int position = mTalonL.getSelectedSensorPosition(0); 
    	return (position >= (Constants.kClimberCargo3rd_EncoderValue - 120));
    }
        
    public boolean atBottom() {
        int position = mTalonL.getSelectedSensorPosition(0); 
    	return (position <= (Constants.kClimberHomeEncoderValue + Constants.kClimberEncoderRange));
    }

    public boolean atDesired() {
        int position = mTalonL.getSelectedSensorPosition(0);
        int hi = (int) mWantedPosition + Constants.kClimberEncoderRange;
        int lo = (int) mWantedPosition - Constants.kClimberEncoderRange;
        boolean result = false;
        if ((position >= lo) && (position <= hi)) {
            result = true;
        }
    	return result;
    }
    */
    
    @Override
    public void zeroSensors() {
        //mTalonL.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, 0, 0);
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing ELEVATOR.-----------------------------------");
        return false;
    }
    
}
