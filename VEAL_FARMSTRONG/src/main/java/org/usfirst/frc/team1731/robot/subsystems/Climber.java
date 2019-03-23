package org.usfirst.frc.team1731.robot.subsystems;

import java.util.Arrays;

import org.usfirst.frc.team1731.lib.util.Util;
import org.usfirst.frc.team1731.lib.util.CheesyDriveHelper;
import org.usfirst.frc.team1731.lib.util.drivers.TalonSRXFactory;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DoubleSolenoid;

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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * 1731 this system controls the climber
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

    private final TalonSRX mTalonLeft;
    private final TalonSRX mTalonRight;
    private DigitalInput latchSensor = new DigitalInput(6);

    private final DoubleSolenoid mDartLatch = Constants.makeDoubleSolenoidForIds(0, Constants.kDartLatch1, Constants.kDartLatch2);
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private Drive mDrive = Drive.getInstance();
    private boolean mDartsHaveDetached = false;


    public Climber() {
        // Left Talon
        mTalonLeft = new TalonSRX(Constants.kClimberTalonLeft);
		/* Factory default hardware to prevent unexpected behavior */
		//mTalonL.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
        mTalonLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		mTalonLeft.setSensorPhase(Constants.kSensorPhase);
		mTalonLeft.setInverted(true);  //old was true

		/* Set relevant frame periods to be at least as fast as periodic rate */
		mTalonLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		mTalonLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		mTalonLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		mTalonLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mTalonLeft.configPeakOutputForward(1.0, Constants.kTimeoutMs);
		mTalonLeft.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalonLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalonLeft.config_kF(Constants.kSlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs);
		mTalonLeft.config_kP(Constants.kSlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs);
		mTalonLeft.config_kI(Constants.kSlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs);
		mTalonLeft.config_kD(Constants.kSlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalonLeft.configMotionCruiseVelocity(Constants.kClimberSlowCruiseVelocity, Constants.kTimeoutMs);
		mTalonLeft.configMotionAcceleration(Constants.kClimberAcceleration, Constants.kTimeoutMs);

		/* Zero the sensor */
        //mTalonL.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalonLeft.set(ControlMode.PercentOutput, 0);

        // FROM WRIST CODE
        //--mTalonL.set(ControlMode.Position, 0);
        //--mTalonL.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        mTalonLeft.configClosedloopRamp(0, Constants.kTimeoutMs);
        //mTalonL.overrideLimitSwitchesEnable(false);
        mTalonLeft.setNeutralMode(NeutralMode.Brake); 
        /*
         * set the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        mTalonLeft.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);

        //Right Talon
        mTalonRight = new TalonSRX(Constants.kClimberTalonRight);
		/* Factory default hardware to prevent unexpected behavior */
		//mTalonR.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
        mTalonRight.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly
		 * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
		 * Phase sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		mTalonRight.setSensorPhase(Constants.kSensorPhase);
		mTalonRight.setInverted(true);  //old was true

		/* Set relevant frame periods to be at least as fast as periodic rate */
		mTalonRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		mTalonRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		mTalonRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		mTalonRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
		mTalonRight.configPeakOutputForward(1.0, Constants.kTimeoutMs);
		mTalonRight.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		mTalonRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mTalonRight.config_kF(Constants.kSlotIdx, Constants.kClimberTalonKF, Constants.kTimeoutMs);
		mTalonRight.config_kP(Constants.kSlotIdx, Constants.kClimberTalonKP, Constants.kTimeoutMs);
		mTalonRight.config_kI(Constants.kSlotIdx, Constants.kClimberTalonKI, Constants.kTimeoutMs);
		mTalonRight.config_kD(Constants.kSlotIdx, Constants.kClimberTalonKD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		mTalonRight.configMotionCruiseVelocity(Constants.kClimberSlowCruiseVelocity, Constants.kTimeoutMs);
		mTalonRight.configMotionAcceleration(Constants.kClimberAcceleration, Constants.kTimeoutMs);

		/* Zero the sensor */
        //mTalonR.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        mTalonRight.set(ControlMode.PercentOutput, 0);

        // FROM WRIST CODE
        //--mTalonR.set(ControlMode.Position, 0);
        //--mTalonR.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 1000);
        mTalonRight.configClosedloopRamp(0, Constants.kTimeoutMs);
        //mTalonR.overrideLimitSwitchesEnable(false);
        mTalonRight.setNeutralMode(NeutralMode.Brake);
        /*
         * set the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        mTalonRight.configAllowableClosedloopError(Constants.kPIDLoopIdx, 3, Constants.kTimeoutMs);        
    }

    public enum SystemState {	
        IDLE,   // stop all motors
        BACKINGUP, // backing up and releasing latch
        LIFTINGNOWHEELS, // lifting without wheels turning
        LIFTINGWITHWHEELS, // lifting with wheels going forward
        RETRACTING // retracting darts
    }

    public enum WantedState {
    	IDLE,   
        CLIMBING // lego lift extend
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
                    case BACKINGUP:
                        newState = handleBackingUp(timestamp);
                        break;
                    case LIFTINGNOWHEELS:
                        newState = handleLiftingNoWheels();
                        break;
                    case LIFTINGWITHWHEELS:
                        newState = handleLiftingWithWheels();
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

    private SystemState defaultStateTransfer(SystemState currentState) {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case CLIMBING:
                return currentState;
        }
        return SystemState.IDLE;
    }
    
    private SystemState handleIdle() {
        mDartLatch.set(Value.kReverse);
       //System.out.println("Climber is running");
        //if (mStateChanged) {
            mTalonLeft.set(ControlMode.MotionMagic, Constants.kClimberRetractedPositionLeft);
            mTalonRight.set(ControlMode.MotionMagic, Constants.kClimberRetractedPositionRight);
        //}
       // SmartDashboard.putBoolean("Latch",latchSensor.get());
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case CLIMBING:
                return SystemState.BACKINGUP;
        }
        return defaultStateTransfer(mSystemState);
    }

    private SystemState handleBackingUp(double timestamp) {
        if (mStateChanged) {
            mDartLatch.set(DoubleSolenoid.Value.kForward); // unlock climber
            //commented-out! WE DON'T NEED TO BACK-UP -----mDrive.setWantClimbBackup(6.0); // drive backwards 6"
            //***NOTE*** Backing up doesn't work anyway!!!
        }

        if (latchSensor.get()) {
            return SystemState.LIFTINGNOWHEELS;
        }
 //        if ((timestamp - mCurrentStateStartTime < Constants.kUnlockClimberTime)) {//mDrive.isBackupComplete()){
//return SystemState.LIFTINGNOWHEELS;
   //     }
      return defaultStateTransfer(mSystemState);
    }

    private SystemState handleLiftingNoWheels() {
        if (mDartsHaveDetached) {
            return SystemState.LIFTINGWITHWHEELS;
        }
        else {
            if (mStateChanged) {
                mTalonLeft.set(ControlMode.MotionMagic, Constants.kClimberExtendedPositionLeft);
                mTalonRight.set(ControlMode.MotionMagic, Constants.kClimberExtendedPositionRight);
            }
            if (mTalonRight.getSelectedSensorPosition() < Constants.kClimberDartsHaveHitTheFloorLeft) {
                 mTalonLeft.configMotionCruiseVelocity(Constants.kClimberSlowCruiseVelocity, Constants.kTimeoutMs);
                 mTalonRight.configMotionCruiseVelocity(Constants.kClimberSlowCruiseVelocity, Constants.kTimeoutMs);
            } else {
                mTalonLeft.configMotionCruiseVelocity(Constants.kClimberMediumCruiseVelocity, Constants.kTimeoutMs);
                mTalonRight.configMotionCruiseVelocity(Constants.kClimberMediumCruiseVelocity, Constants.kTimeoutMs);
            }
        //    System.out.println("in liftingnowheels");
            if(mTalonRight.getSelectedSensorPosition() > Constants.kClimberStartWheelsLeft) { 
                return SystemState.LIFTINGWITHWHEELS;
            }
            return defaultStateTransfer(mSystemState);
         }
}

    private SystemState handleLiftingWithWheels() {
        mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(1.0, 0.0, false, true));
        if(mTalonRight.getSelectedSensorPosition() > Constants.kClimberDartsHaveDetachedRight  ){
         mDartsHaveDetached = true;
        }
      //  System.out.println("------------------in lifting with wheels");
        if(mTalonRight.getSelectedSensorPosition() > 0.95*Constants.kClimberExtendedPositionRight ){
            return SystemState.RETRACTING;
        }
        return defaultStateTransfer(mSystemState);
    }

    private SystemState handleRetracting() {
       // System.out.println("RETRACTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (mStateChanged) {
            mTalonLeft.configMotionCruiseVelocity(Constants.kClimberFastCruiseVelocity, Constants.kTimeoutMs);
            mTalonRight.configMotionCruiseVelocity(Constants.kClimberFastCruiseVelocity, Constants.kTimeoutMs);
            mTalonLeft.set(ControlMode.MotionMagic, Constants.kClimberRetractedPositionLeft);
            mTalonRight.set(ControlMode.MotionMagic, Constants.kClimberRetractedPositionRight);
        }
        return defaultStateTransfer(mSystemState);
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mWantedState = state;
        }
    }

    @Override
    public void outputToSmartDashboard() {
   //     SmartDashboard.putString("ClimbSysState", mSystemState.name());
   //     SmartDashboard.putString("ClimbWantState", mWantedState.name());
   //     SmartDashboard.putNumber("ClimbCurPosLeft", mTalonL.getSelectedSensorPosition(0));
   //     SmartDashboard.putNumber("ClimbCurPosRight", mTalonR.getSelectedSensorPosition(0));
    }

    public void resetLift() {
        mDartsHaveDetached = false;
    }

    @Override
    public void stop() {
        // mVictor.set(0);
        setWantedState(WantedState.IDLE);
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
        //mTalonLeft.setSelectedSensorPosition(Constants.kClimberHomeEncoderValue, 0, 0);
    }
    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing CLIMBER.-----------------------------------");
        return false;
    }
    
}
