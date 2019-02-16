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

    private final TalonSRX mTalon;
    //private final Solenoid mOverTop1;
    //private final Solenoid mOverTop2;
    
    public Climber() {
        mTalon = new TalonSRX(Constants.kClimberTalon);
		/* Factory default hardware to prevent unexpected behavior */
		mTalon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
        mTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		mTalon.setSensorPhase(Constants.kSensorPhase);
		mTalon.setInverted(true);  //old was true

		/* Set relevant frame periods to be at least as fast as periodic rate */
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		mTalon.configNominalOutputForward(0, Constants.kTimeoutMs);
		mTalon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mTalon.configPeakOutputForward(0.3, Constants.kTimeoutMs);
		mTalon.configPeakOutputReverse(-0.1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalon.config_kF(Constants.kSlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs);
		mTalon.config_kP(Constants.kSlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs);
		mTalon.config_kI(Constants.kSlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs);
		mTalon.config_kD(Constants.kSlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalon.configMotionCruiseVelocity(Constants.kClimberCruiseVelocity, Constants.kTimeoutMs);
		mTalon.configMotionAcceleration(Constants.kClimberAcceleration, Constants.kTimeoutMs);

		/* Zero the sensor */
        //mTalon.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalon.set(ControlMode.PercentOutput, 0);

        // FROM WRIST CODE
        //--mTalon.set(ControlMode.Position, 0);
        //--mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        //mTalon.configClosedloopRamp(0, Constants.kTimeoutMs);
        //mTalon.overrideLimitSwitchesEnable(false);
        //mTalon.setNeutralMode(NeutralMode.Brake);
        /* choose based on what direction you want forward/positive to be.
         * This does not affect sensor phase. */ 
        //mTalon.setInverted(true); //Constants.kMotorInvert);
        /*
         * set the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        mTalon.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);
    }
    /*
    public xClimber() {
        ///mTalon = new TalonSRX(Constants.kClimberTalon);
        ///mTalon.set(ControlMode.Position, 0);
        mTalon.configVelocityMeasurementWindow(10, 0);
        mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
        ///mTalon.selectProfileSlot(0, 0);
        ///mTalon.config_kP(Constants.SlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs );
        ///mTalon.config_kI(Constants.SlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs );
        ///mTalon.config_kD(Constants.SlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);
        ///mTalon.config_kF(Constants.SlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs );
        ///mTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        ///mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        mTalon.configClosedloopRamp(0, Constants.kTimeoutMs);
        ///mTalon.setSelectedSensorPosition(0, 0, 10); //-1793, 0, 10);
        mTalon.overrideLimitSwitchesEnable(false);
        
        / * choose to ensure sensor is positive when output is positive * /
        ///mTalon.setSensorPhase(Constants.kSensorPhase);

        /* choose based on what direction you want forward/positive to be.
         * This does not affect sensor phase. * / 
        ////mTalon.setInverted(true); //Constants.kMotorInvert);

        / * set the peak and nominal outputs, 12V means full * /
        ///mTalon.configNominalOutputForward(.5, Constants.kTimeoutMs);
        ///mTalon.configNominalOutputReverse(.9, Constants.kTimeoutMs);
        ///mTalon.configPeakOutputForward(1.0, Constants.kTimeoutMs);
        ///mTalon.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
        
        mTalon.configAllowableClosedloopError(Constants.kPIDLoopIdx, 50, Constants.kTimeoutMs);
    }
    */
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
                //mTalon.setSelectedSensorPosition(0, 0, 10);                
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
            mTalon.set(ControlMode.PercentOutput, 0);
        }
        return defaultStateTransfer();
    }

    private SystemState handleExtending() {
        if (mStateChanged) {
            mTalon.set(ControlMode.PercentOutput, Constants.kClimberExtendPercent);
        }
        //mPositionChanged = true;
        //mWantedPosition = Constants.kClimberHomeEncoderValue;
        //wasCalibrated = true;
        return defaultStateTransfer();
    }

    private SystemState handleRetracting() {
        if (mStateChanged) {
            mTalon.set(ControlMode.PercentOutput, Constants.kClimberRetractPercent);
        }
        //mPositionChanged = true;
        //mWantedPosition = Constants.kClimberHomeEncoderValue;
        //mTalon.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, 0, 0);
        //wasCalibrated = true;
        return defaultStateTransfer();
    }
    /*
    public synchronized void setWantedPosition(double position) {
        
        if (position != mWantedPosition) {
            if ((mWantedPosition >= Constants.kClimberHomeEncoderValue) && 
                    (mWantedPosition < Constants.kClimberTopEncoderValue)) {
                mWantedPosition = position;
                mPositionChanged = true;
            }
        }
    }
    */
    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
            //DriverStation.reportError("Climber WantedState: " + mWantedState, false);
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("ClimbSysState", mSystemState.name());
        SmartDashboard.putString("ClimbWantState", mWantedState.name());
    }

    @Override
    public void stop() {
        // mVictor.set(0);
        setWantedState(WantedState.IDLE);
    }
    /*
    public synchronized int getCurrentPosition() {
    	return mTalon.getSelectedSensorPosition(0);
    }
    
	public boolean atTop() {
		int position = mTalon.getSelectedSensorPosition(0); 
    	return (position >= (Constants.kClimberCargo3rd_EncoderValue - 120));
    }
        
    public boolean atBottom() {
        int position = mTalon.getSelectedSensorPosition(0); 
    	return (position <= (Constants.kClimberHomeEncoderValue + Constants.kClimberEncoderRange));
    }

    public boolean atDesired() {
        int position = mTalon.getSelectedSensorPosition(0);
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
        //mTalon.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, 0, 0);
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing ELEVATOR.-----------------------------------");
        return false;
    }

    //private boolean checkRevSwitch() {
    //    boolean revSwitch = mTalon.getSensorCollection().isRevLimitSwitchClosed();
    //    if (revSwitch) {
    //        if (!mRevSwitchSet) {
    //            mTalon.setSelectedSensorPosition(-1 * (int)Constants.kClimberHomeEncoderValue, 0, 10);
    //            mRevSwitchSet = true;
    //        }
    //    } else {
    //        mRevSwitchSet = false;
    //    }
    //    
    //    return revSwitch;
    //}
    
}
