package org.usfirst.frc.team1731.robot.subsystems;

import java.util.Optional;

import org.usfirst.frc.team1731.lib.util.CircularBuffer;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.drivers.RevRoboticsAirPressureSensor;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.Constants.GRABBER_POSITION;
import org.usfirst.frc.team1731.robot.Constants.ELEVATOR_POSITION;
import org.usfirst.frc.team1731.robot.Robot;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.loops.Loop;
import org.usfirst.frc.team1731.robot.loops.Looper;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * 
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * 
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 * 
 * @see Intake
 * @see Hopper
 * @see Elevator
 * @see LED
 * @see Subsystem
 */
public class Superstructure extends Subsystem {

    public enum WantedWristPosition {
    	CARGOPICKUP,   
        STRAIGHTAHEAD, // moving
        SHOOTHIGH,
        STARTINGPOSITION
    }
    
	static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private final Elevator mElevator = Elevator.getInstance();

    private final Intake mIntake = Intake.getInstance();

    private final DoubleSolenoid mTopRoller = Constants.makeDoubleSolenoidForIds(1, Constants.kTopRoller1, Constants.kTopRoller2);
    private final DoubleSolenoid mBeakSwinger = Constants.makeDoubleSolenoidForIds(0, Constants.kBeakSwinger1, Constants.kBeakSwinger2);

//DEBUG ONLY - NOT FOR COMPETITION - PUT THIS BACK IN!!!!!!!!!
    private final DoubleSolenoid mBeakLips = Constants.makeDoubleSolenoidForIds(1, Constants.kBeakOpener1, Constants.kBeakOpener2);
   // private final DoubleSolenoid mBeakLips = Constants.makeDoubleSolenoidForIds(0, 3, 2);
    
    
    private final DoubleSolenoid mMustache = Constants.makeDoubleSolenoidForIds(0, Constants.kMustache1, Constants.kMustache2);
    private final DoubleSolenoid mRotateWristShort = Constants.makeDoubleSolenoidForIds(1, Constants.kRotateWristShort1, Constants.kRotateWristShort2);
    private final DoubleSolenoid mRotateWristLong = Constants.makeDoubleSolenoidForIds(1, Constants.kRotateWristLong1, Constants.kRotateWristLong2); 

    //private final Compressor mCompressor = new Compressor(0);
    //private final RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    private final Climber mClimber = Climber.getInstance();
    
   // private final Wrist mWrist = Wrist.getInstance();

    // Superstructure doesn't own the drive, but needs to access it
    private final Drive mDrive = Drive.getInstance();

    // Intenal state of the system
    public enum SystemState {
        IDLE,
        WAITING_FOR_LOW_POSITION,
        WAITING_FOR_HIGH_POSITION,
        WAITING_FOR_POWERCUBE_INTAKE,
        CLIMBING,
        CALIBRATINGUP,
        CALIBRATINGDOWN,
        SPITTING,
        WAITING_FOR_ROTATE,
        SPITTING_OUT_TOP, 
        RETURNINGFROMINTAKE,
        RETURNING_HOME,
        ELEVATOR_TRACKING,
        CARGO_CAPTURED,
        CAPTURING_CARGO,
        EJECTING_HATCH,
        EJECTING_CARGO,
        CAPTURING_HATCH,
        HATCH_CAPTURED,
        STARTINGCONFIGURATION
    };

    // Desired function from user
    public enum WantedState {
        IDLE,
        CLIMBINGUP, 
        CLIMBINGDOWN, 
        INTAKING,
        AUTOINTAKING,
        SPITTING,
        CALIBRATINGDOWN, 
        CALIBRATINGUP,
        OVERTHETOP,
        ELEVATOR_TRACKING,
        HATCH_CAPTURED,
        EJECTING_CARGO,
        EJECTING_HATCH,
        CARGO_CAPTURED,
        STARTINGCONFIGURATION
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;
    private double mWantedElevatorPosition = Constants.kElevatorHomeEncoderValue;
    private double mIntakeOutput = 0;
    private Loop mLoop = new Loop() {

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mWantStateChangeStartTime;

        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.IDLE;
                mCurrentStateStartTime = timestamp;
                mWantStateChangeStartTime = timestamp;
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState = mSystemState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle(mStateChanged);
                    break;
                case CLIMBING:
                    newState = handleClimberExtending();
                    break;
                case WAITING_FOR_LOW_POSITION:
                    newState = handleWaitingForLowPosition();
                    break;
                case WAITING_FOR_HIGH_POSITION:
                    newState = handleWaitingForHightPosition();
                    break;
                case WAITING_FOR_POWERCUBE_INTAKE:
                    newState = waitingForPowerCubeIntake();
                    break;
                case CALIBRATINGUP:
                    newState = handleCalibrationUp();
                    break;
                case CALIBRATINGDOWN:
                    newState = handleCalibrationDown();
                    break;
                case STARTINGCONFIGURATION:
                    newState = handleStartingConfiguration();
                    break;
                case SPITTING:
                    newState = handleSpitting();
                    break;
                case WAITING_FOR_ROTATE:
                    newState = handleWaitingForRotate(timestamp);
                    break;
                case SPITTING_OUT_TOP:
                    newState = handleSpittingOutTop();
                    break;
                case ELEVATOR_TRACKING:
                    newState = handleElevatorTracking(timestamp);
                    break;
                case RETURNING_HOME:
                    newState = handleReturningHome();
                    break;
                case CARGO_CAPTURED:
                    newState = handleCargoCapture();
                    break;
                case HATCH_CAPTURED:
                    newState = handleHatchCapture();
                    break;
                case EJECTING_HATCH:
                    newState = handleEjectingHatch();
                    break;
                case EJECTING_CARGO:
                    newState = handleEjectingCargo();
                    break;
                default:
                    newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure state " + mSystemState + " to " + newState + " Timestamp: "
                            + Timer.getFPGATimestamp());
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        private SystemState handleEjectingCargo() {
            mBeakSwinger.set(DoubleSolenoid.Value.kReverse);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kReverse);
            mMustache.set(DoubleSolenoid.Value.kReverse);
            mIntake.setWantedState(Intake.WantedState.SPITTING);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleStartingConfiguration(){
            mBeakSwinger.set(DoubleSolenoid.Value.kForward);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kForward);
            mMustache.set(DoubleSolenoid.Value.kReverse);
            mClimber.setWantedState(Climber.WantedState.IDLE);
           // mWrist.setWantedPosition(WristPositions.STARTINGPOSITION);
            seWristtWantedPosition(WantedWristPosition.STARTINGPOSITION);

            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleCargoCapture() {
            mBeakSwinger.set(DoubleSolenoid.Value.kReverse);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kForward);
            mMustache.set(DoubleSolenoid.Value.kReverse);
            mIntake.setWantedState(Intake.WantedState.INTAKING);
            seWristtWantedPosition(WantedWristPosition.CARGOPICKUP);
            setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_CARGO_PICKUP);
            mElevator.setWantedPosition(Constants.kElevatorBallPickup_EncoderValue);

            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                mTopRoller.set(DoubleSolenoid.Value.kReverse);
                seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleEjectingHatch() {
            mBeakSwinger.set(DoubleSolenoid.Value.kForward);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kReverse);
            mMustache.set(DoubleSolenoid.Value.kForward);

            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleHatchCapture() {            
            mBeakSwinger.set(DoubleSolenoid.Value.kForward);
            mBeakLips.set(DoubleSolenoid.Value.kReverse);
            mTopRoller.set(DoubleSolenoid.Value.kReverse);
            mMustache.set(DoubleSolenoid.Value.kReverse);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case ELEVATOR_TRACKING:
            
                mBeakLips.set(DoubleSolenoid.Value.kForward);
                return SystemState.ELEVATOR_TRACKING;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleElevatorTracking(double timestamp) {
        	mElevator.setWantedPosition(mWantedElevatorPosition);
            mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
            if (mIntake.getSystemState() != Intake.SystemState.HOLDING) {
                mIntake.setWantedState(Intake.WantedState.IDLE);
            }
            mClimber.setWantedState(Climber.WantedState.IDLE);
            if ((timestamp - mCurrentStateStartTime  > 1.0)) {
                mMustache.set(DoubleSolenoid.Value.kReverse);
            }
            
            
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }
        
        private SystemState handleReturningHome() {
        	//mElevator.setWantedPosition(0);
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
            mIntake.setWantedState(Intake.WantedState.IDLE);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.RETURNING_HOME;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.WAITING_FOR_HIGH_POSITION;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;    
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleSpittingOutTop() {
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
        	mIntake.setWantedState(Intake.WantedState.SPITTING);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleWaitingForRotate(double timestamp) {
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
        	mIntake.setIdle();      
       
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                if ((timestamp - mCurrentStateStartTime < Constants.kRotateTime)) {
                	return SystemState.SPITTING_OUT_TOP;
                }
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleSpitting() {
        	//mElevator.setWantedPosition(Constants.kElevatorHomeEncoderValue);
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
        	mIntake.setWantedState(Intake.WantedState.SPITTING);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
             case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;    
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleCalibrationDown() {
            mElevator.setWantedState(Elevator.WantedState.CALIBRATINGDOWN);
            mWantedElevatorPosition = Constants.kElevatorHomeEncoderValue;
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleCalibrationUp() {
            mElevator.setWantedState(Elevator.WantedState.CALIBRATINGUP);
            mWantedElevatorPosition = Constants.kElevatorHomeEncoderValue;
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;    
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState waitingForPowerCubeIntake() {

       	    mIntake.setWantedState(Intake.WantedState.INTAKING);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
            	//if (mIntake.gotCube()) {
            	//	return SystemState.RETURNING_HOME;
            	//}
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;    
            default:
                return SystemState.IDLE;
            }
        }

		private SystemState handleWaitingForHightPosition() {
			return SystemState.IDLE;
		}

		private SystemState handleWaitingForLowPosition() {
        	mElevator.setWantedState(Elevator.WantedState.ELEVATORTRACKING);
        	mIntake.setWantedState(Intake.WantedState.INTAKING);
        	
            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:{
            	if (mElevator.atBottom())
            		return SystemState.WAITING_FOR_POWERCUBE_INTAKE; 
            	else  
                return SystemState.WAITING_FOR_LOW_POSITION;
            	}
            	
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case OVERTHETOP:
                return SystemState.SPITTING_OUT_TOP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            case HATCH_CAPTURED:
                return SystemState.HATCH_CAPTURED;
            case EJECTING_HATCH:
                return SystemState.EJECTING_HATCH;
            case CARGO_CAPTURED:
                return SystemState.CARGO_CAPTURED;
            case EJECTING_CARGO:
                return SystemState.EJECTING_CARGO;      
            default:
                return SystemState.IDLE;
            }
        }

        private SystemState handleClimberExtending() {
            mClimber.setWantedState(Climber.WantedState.CLIMBING);

            switch (mWantedState) {
            case CLIMBINGUP:
                return SystemState.CLIMBING;
            case AUTOINTAKING:
                return SystemState.WAITING_FOR_LOW_POSITION;
            case INTAKING:
                return SystemState.WAITING_FOR_POWERCUBE_INTAKE;
            case SPITTING:
                return SystemState.SPITTING;
            case CALIBRATINGDOWN:
                return SystemState.CALIBRATINGDOWN;
            case STARTINGCONFIGURATION:
                return SystemState.STARTINGCONFIGURATION;
            case CALIBRATINGUP:
                return SystemState.CALIBRATINGUP;
            case ELEVATOR_TRACKING:
                return SystemState.ELEVATOR_TRACKING;
            default:
                return SystemState.IDLE;
            }
        }

		@Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private SystemState handleIdle(boolean stateChanged) {
        if (stateChanged) {
            stop();
            mMustache.set(DoubleSolenoid.Value.kReverse);
            mElevator.setWantedState(Elevator.WantedState.IDLE);
            mIntake.setWantedState(Intake.WantedState.IDLE);
            mClimber.setWantedState(Climber.WantedState.IDLE);
        }
        
        switch (mWantedState) {
        case CLIMBINGUP:
            return SystemState.CLIMBING;
        case AUTOINTAKING:
            return SystemState.WAITING_FOR_LOW_POSITION;
        case INTAKING:
            return SystemState.WAITING_FOR_POWERCUBE_INTAKE;      
        case SPITTING:
            return SystemState.SPITTING;
        case CALIBRATINGDOWN:
            return SystemState.CALIBRATINGDOWN;
        case STARTINGCONFIGURATION:
            return SystemState.STARTINGCONFIGURATION;
        case CALIBRATINGUP:
            return SystemState.CALIBRATINGUP;
        case OVERTHETOP:
            return SystemState.WAITING_FOR_HIGH_POSITION;
        case ELEVATOR_TRACKING:
            return SystemState.ELEVATOR_TRACKING;
        case HATCH_CAPTURED:
            return SystemState.HATCH_CAPTURED;
        case EJECTING_HATCH:
            return SystemState.EJECTING_HATCH; 
        case CARGO_CAPTURED:
            return SystemState.CARGO_CAPTURED;
        case EJECTING_CARGO:
            return SystemState.EJECTING_CARGO;     
        default:
            return SystemState.IDLE;
        }
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public void outputToSmartDashboard() {
        //SmartDashboard.putNumber("Air Pressure psi", mAirPressureSensor.getAirPressurePsi());
        SmartDashboard.putString("Sys State", mSystemState.name());
        SmartDashboard.putNumber("IntakeOutput", mIntakeOutput);
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public void setWantedElevatorPosition(ELEVATOR_POSITION position) {
     //   System.out.println("INELEVATORSETWANTEDPOSITION******************************************************");
        boolean cargo = mIntake.hasCargo();
        double encoderValue = Constants.kElevatorHomeEncoderValue;
        if (cargo) {
            switch (position) {
                case ELEVATOR_FLOOR:
                    encoderValue = (double) Constants.kElevatorCargoFloor_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_2ND:
                    encoderValue = (double) Constants.kElevatorCargo2nd_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_3RD:
                    encoderValue = (double) Constants.kElevatorCargo3rd_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.SHOOTHIGH);
                    break;
                case ELEVATOR_SHIP:
                    encoderValue = (double) Constants.kElevatorCargoShip_EncoderValue;
                    break;
                case ELEVATOR_CARGO_PICKUP:
                    encoderValue = (double) Constants.kElevatorBallPickup_EncoderValue;
                    break;
            }
        } else {
            switch (position) {
                case ELEVATOR_FLOOR:
                    encoderValue = (double) Constants.kElevatorHatchFloor_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_2ND:
                    encoderValue = (double) Constants.kElevatorHatch2nd_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_3RD:
                    encoderValue = (double) Constants.kElevatorHatch3rd_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_SHIP:
                    encoderValue = (double) Constants.kElevatorHatchShip_EncoderValue;
                    seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
                    break;
                case ELEVATOR_CARGO_PICKUP:
                    encoderValue = (double) Constants.kElevatorBallPickup_EncoderValue;
                    break;
            }
        }
        mWantedElevatorPosition = encoderValue;
    }

    public synchronized void seWristtWantedPosition(WantedWristPosition position) {

        switch (position) {
            case CARGOPICKUP:
                mRotateWristShort.set(DoubleSolenoid.Value.kReverse);
                mRotateWristLong.set(DoubleSolenoid.Value.kReverse);
                break;
            case STRAIGHTAHEAD:
                mRotateWristShort.set(DoubleSolenoid.Value.kForward);
                mRotateWristLong.set(DoubleSolenoid.Value.kReverse);
                break;
             case SHOOTHIGH:
                mRotateWristShort.set(DoubleSolenoid.Value.kReverse);
                mRotateWristLong.set(DoubleSolenoid.Value.kForward);
                break;
             case STARTINGPOSITION:
                mRotateWristShort.set(DoubleSolenoid.Value.kForward);
                mRotateWristLong.set(DoubleSolenoid.Value.kForward);
                break;
            default:
                mRotateWristShort.set(DoubleSolenoid.Value.kForward);
                mRotateWristLong.set(DoubleSolenoid.Value.kForward);
        }

    }

    public void closeBeak() {
        mBeakLips.set(DoubleSolenoid.Value.kReverse);

    }
    public void openBeak() {
        mBeakLips.set(DoubleSolenoid.Value.kForward);

    }

    public void prepareToPickupHatch() {
        mBeakSwinger.set(DoubleSolenoid.Value.kForward);
        mBeakLips.set(DoubleSolenoid.Value.kReverse);
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        mMustache.set(DoubleSolenoid.Value.kReverse);
    }
   

    public void ejectHatch() {
        mBeakSwinger.set(DoubleSolenoid.Value.kForward);
        mBeakLips.set(DoubleSolenoid.Value.kReverse);
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        mMustache.set(DoubleSolenoid.Value.kForward);
    }

    public void uproller() {
        mTopRoller.set(DoubleSolenoid.Value.kReverse);
        seWristtWantedPosition(WantedWristPosition.STRAIGHTAHEAD);
    }

    public void setOverrideCompressor(boolean force_off) {
    }

    public void reloadConstants() {
    }

}
