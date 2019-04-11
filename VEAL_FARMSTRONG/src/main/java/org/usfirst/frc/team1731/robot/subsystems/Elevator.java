package org.usfirst.frc.team1731.robot.subsystems;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * 1731 this system controls the elevator
 * 
 * @see Subsystem.java
 */

@SuppressWarnings("unused")
public class Elevator extends Subsystem {

    private static Elevator sInstance = null;
    private static boolean isReset;

    public static Elevator getInstance() {
        if (sInstance == null) {
            sInstance = new Elevator();
        }
        return sInstance;
    }

    private final TalonSRX mTalon;
    
    public Elevator() {
        mTalon = new TalonSRX(Constants.kElevatorTalon);
		/* Factory default hardware to prevent unexpected behavior */
		mTalon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
     //   mTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

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
		mTalon.configPeakOutputForward(1, Constants.kTimeoutMs);
		mTalon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalon.config_kF(Constants.kSlotIdx, Constants.kElevatorTalonKF, Constants.kTimeoutMs);
		mTalon.config_kP(Constants.kSlotIdx, Constants.kElevatorTalonKP, Constants.kTimeoutMs);
		mTalon.config_kI(Constants.kSlotIdx, Constants.kElevatorTalonKI, Constants.kTimeoutMs);
		mTalon.config_kD(Constants.kSlotIdx, Constants.kElevatorTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalon.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kTimeoutMs);
		mTalon.configMotionAcceleration(Constants.kElevatorAcceleration, Constants.kTimeoutMs);
        mTalon.configAllowableClosedloopError(Constants.kPIDLoopIdx, 1000, Constants.kTimeoutMs);
		/* Zero the sensor */
        mTalon.setSelectedSensorPosition(Constants.kElevatorHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        //mTalon.set(ControlMode.PercentOutput, 0);
    }
    /*
    public xElevator() {
        ///mTalon = new TalonSRX(Constants.kElevatorTalon);
        ///mTalon.set(ControlMode.Position, 0);
        mTalon.configVelocityMeasurementWindow(10, 0);
        mTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
        ///mTalon.selectProfileSlot(0, 0);
        ///mTalon.config_kP(Constants.SlotIdx, Constants.kElevatorTalonKP, Constants.kTimeoutMs );
        ///mTalon.config_kI(Constants.SlotIdx, Constants.kElevatorTalonKI, Constants.kTimeoutMs );
        ///mTalon.config_kD(Constants.SlotIdx, Constants.kElevatorTalonKD, Constants.kTimeoutMs);
        ///mTalon.config_kF(Constants.SlotIdx, Constants.kElevatorTalonKF, Constants.kTimeoutMs );
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
        ELEVATORTRACKING, // moving
        CALIBRATINGUP,
        CALIBRATINGDOWN,
    }

    public enum WantedState {
    	IDLE,   
        ELEVATORTRACKING, // moving
        CALIBRATINGUP,
        CALIBRATINGDOWN,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private double mWantedPosition = 0;
    private boolean mStateChanged = false;
    private boolean mPositionChanged = false;
    private boolean wasCalibrated = false;
    //private boolean mRevSwitchSet = false;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Elevator.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mPositionChanged = false;
                mWantedPosition = 0;
                mCurrentStateStartTime = timestamp;
                if(!isReset){
                    mTalon.setSelectedSensorPosition(1000, 0, 10);                
                    isReset = true;
                }
              //  DriverStation.reportError("Elevator SystemState: " + mSystemState, false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
   	
        	synchronized (Elevator.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case ELEVATORTRACKING:
                        newState = handleElevatorTracking();
                        break;
                    case CALIBRATINGUP:
                        newState = handleCalibratingUp();
                        break;
                    case CALIBRATINGDOWN:
                        newState = handleCalibratingDown();
                        break;
                    default:
                        newState = SystemState.IDLE;                    
                }

                if (newState != mSystemState) {
                    System.out.println("Elevator state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    //DriverStation.reportWarning("Elevator SystemState: " + mSystemState, false);
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
            case ELEVATORTRACKING:
                return SystemState.ELEVATORTRACKING;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;

            default:
                return SystemState.IDLE;
        }
    }
    
    private SystemState handleIdle() {
        if (mStateChanged) {
  //          System.out.println("ININDLE@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
            mTalon.set(ControlMode.PercentOutput, 0);
        }
        return defaultStateTransfer();
    }

    private SystemState handleElevatorTracking() {
        if (wasCalibrated) {
            wasCalibrated = false;
            zeroSensors();
        }
        if (mPositionChanged) {
            mTalon.set(ControlMode.MotionMagic, mWantedPosition);
            mPositionChanged = false;
        }
    	
        // int curPos = mTalon.getSelectedSensorPosition(0);
        // System.out.println("Pos:" + mWantedPosition + ", EncVal: " + curPos);

    		//if (checkRevSwitch()) {
            //    if (nextPos < -1 * (int)Constants.kElevatorBottomEncoderValue) {
    		//        nextPos = -1 * (int)Constants.kElevatorBottomEncoderValue;
            //    }
    		//}

	    return defaultStateTransfer();
    }

    private SystemState handleCalibratingDown() {
        if (mStateChanged) {
            mTalon.set(ControlMode.PercentOutput, Constants.kElevatorCalibrateDown);
        }
        mPositionChanged = true;
        mWantedPosition = Constants.kElevatorHomeEncoderValue;
        mTalon.setSelectedSensorPosition(-3000, 0, 0);
        wasCalibrated = true;
        return defaultStateTransfer();
    }

    private SystemState handleCalibratingUp() {
        if (mStateChanged) {
            mTalon.set(ControlMode.PercentOutput, Constants.kElevatorCalibrateUp);
        }
        mPositionChanged = true;
        mWantedPosition = Constants.kElevatorHomeEncoderValue;
        mTalon.setSelectedSensorPosition(Constants.kElevatorHomeEncoderValue, 0, 0);
        wasCalibrated = true;
        return defaultStateTransfer();
    }

    public synchronized void setWantedPosition(double position) {
        
        if (position != mWantedPosition) {
            if ((mWantedPosition >= Constants.kElevatorHomeEncoderValue) && 
                    (mWantedPosition < Constants.kElevatorTopEncoderValue)) {
                mWantedPosition = position;
                mPositionChanged = true;
//                System.out.println("Elevator wanted position changed to: " + position);
            }
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
            //DriverStation.reportError("Elevator WantedState: " + mWantedState, false);
        }
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("ElevSysState", mSystemState.name()); // .ordinal());
        SmartDashboard.putString("ElevWantState", mWantedState.name());
        //SmartDashboard.putNumber("ElevWantState", (double)mWantedState.ordinal());
        SmartDashboard.putNumber("ElevWantPos", mWantedPosition);
        SmartDashboard.putNumber("ElevCurPos", mTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("ElevQuadPos", mTalon.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putBoolean("ElevPosChange", mPositionChanged);
        //SmartDashboard.putBoolean("ElevLastRevSw", mRevSwitchSet);
    }

    @Override
    public void stop() {
        // mVictor.set(0);
        setWantedState(WantedState.IDLE);
    }

    public synchronized int getCurrentPosition() {
    	return mTalon.getSelectedSensorPosition(0);
    }
    
	public boolean atTop() {
		int position = mTalon.getSelectedSensorPosition(0); 
    	return (position >= (Constants.kElevatorCargo3rd_EncoderValue - 120));
    }
        
    public boolean atBottom() {
        int position = mTalon.getSelectedSensorPosition(0); 
    	return (position <= (Constants.kElevatorHomeEncoderValue + Constants.kElevatorEncoderRange));
    }

    public boolean atDesired() {
        int position = mTalon.getSelectedSensorPosition(0);
        int hi = (int) mWantedPosition + Constants.kElevatorEncoderRange;
        int lo = (int) mWantedPosition - Constants.kElevatorEncoderRange;
        boolean result = false;
        if ((position >= lo) && (position <= hi)) {
            result = true;
        }
    	return result;
    }
    
    @Override
    public void zeroSensors() {
     //  mTalon.setSelectedSensorPosition(Constants.kElevatorHomeEncoderValue, 0, 0);
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
    //            mTalon.setSelectedSensorPosition(-1 * (int)Constants.kElevatorHomeEncoderValue, 0, 10);
    //            mRevSwitchSet = true;
    //        }
    //    } else {
    //        mRevSwitchSet = false;
    //    }
    //    
    //    return revSwitch;
    //}
    
}
