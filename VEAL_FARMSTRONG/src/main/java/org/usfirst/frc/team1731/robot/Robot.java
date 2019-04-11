package org.usfirst.frc.team1731.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.usfirst.frc.team1731.lib.util.CheesyDriveHelper;
import org.usfirst.frc.team1731.lib.util.CrashTracker;
import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.InterpolatingTreeMap;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.robot.Constants.ELEVATOR_POSITION;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeExecuter;
import org.usfirst.frc.team1731.robot.auto.modes.LeftFeedStationToRocketRearMode;
import org.usfirst.frc.team1731.robot.auto.modes.LeftRocketFrontToFeedStationMode;
import org.usfirst.frc.team1731.robot.auto.modes.LeftFeedStationToRocketFrontMode;
import org.usfirst.frc.team1731.robot.auto.modes.LeftRocketRearToFeedStationMode;
import org.usfirst.frc.team1731.robot.auto.modes.PlacePanel;
import org.usfirst.frc.team1731.robot.auto.modes.LeftLevel1ToRocketRearMode;
import org.usfirst.frc.team1731.robot.auto.modes.LeftLevel1ToCargoL1Mode;
import org.usfirst.frc.team1731.robot.auto.modes.LeftCargoL1ToFeederStationMode;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_1;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_10;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_2;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_3;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_4;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_5;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_6;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_7;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_8;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_9;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_A;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_B;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_C;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_D;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_E;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_F;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_G;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_H;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_I;
import org.usfirst.frc.team1731.robot.auto.modes.spacey.Mode_J;
import org.usfirst.frc.team1731.robot.loops.Looper;
import org.usfirst.frc.team1731.robot.loops.RobotStateEstimator;
import org.usfirst.frc.team1731.robot.loops.VisionCamProcessor;
import org.usfirst.frc.team1731.robot.loops.JevoisVisionProcessor;
import org.usfirst.frc.team1731.robot.subsystems.ConnectionMonitor;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Elevator;

import org.usfirst.frc.team1731.robot.subsystems.Intake;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.vision.JevoisVisionServer;
import org.usfirst.frc.team1731.robot.subsystems.Climber;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;


/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 * 
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 * 
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 * 
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
    private DigitalOutput leftRightCameraControl;
    private DigitalOutput greenLEDRingLight;
		
	private static final String AUTO_CODE = "AutoCode";
    private static Map<String, AutoModeBase> AUTO_MODES; // modes defined in Mark's "BIBLE"
    
    private static AutoModeBase newLeftRocketRearToFeedStationMode  = new LeftRocketRearToFeedStationMode();
    private static AutoModeBase newLeftFeedStationToRocketRearMode  = new LeftFeedStationToRocketRearMode();
    private static AutoModeBase newLeftRocketFrontToFeedStationMode = new LeftRocketFrontToFeedStationMode();
    private static AutoModeBase newLeftFeedStationToRocketFrontMode = new LeftFeedStationToRocketFrontMode();
    private static AutoModeBase newLeftLevel1ToCargoL1Mode          = new LeftLevel1ToCargoL1Mode();
    private static AutoModeBase newLeftCargoL1ToFeederStationMode   = new LeftCargoL1ToFeederStationMode();
    private static AutoModeBase newLeftLevel1ToRocketRearMode       = new LeftLevel1ToRocketRearMode();

	static {
		initAutoModes();
	}
	
	// Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private Climber mClimber = Climber.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    private RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
 //  private VisionCamProcessor mVisionCamProcessor = VisionCamProcessor.getInstance();
    private JevoisVisionProcessor mVisionCamProcessor = JevoisVisionProcessor.getInstance();

    private AutoModeBase[] autoModesToExecute;

    private boolean joystickAxesAreReversed;
    private boolean driveSpeedIsToggled;
    private boolean camerasAreReversed;
    private boolean tractorIndicator = Boolean.FALSE;  
    private boolean mLoopersAreRunning = false;       
    
    private UsbCamera cameraFront;
    //private UsbCamera cameraBack;
    //private UsbCamera selectedCamera;
    private DigitalOutput arduinoLed0;
    private DigitalOutput arduinoLed1;
    private DigitalOutput arduinoLed2;

    private Boolean invertCameraPrevious = Boolean.FALSE;
    private Boolean mTractorBeamPickupSelected = false;
    private Boolean mTractorBeamHatchSelected = false;
    private VideoSink videoSink;

    private String autoCode = Constants.kDefaultAutoMode;
    private JevoisVisionServer mVisionServer = JevoisVisionServer.getInstance();
    
    private final SubsystemManager mSubsystemManager = new SubsystemManager(
                           Arrays.asList(
                                         Drive.getInstance(),
                                         Superstructure.getInstance(),
                                         Elevator.getInstance(),
                                         Intake.getInstance(),
                                         Climber.getInstance()
                                         ));

    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);

    private Double mTractorBeamGain = 1.0;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    private DigitalInput tapeSensor;

    //private SerialPort visionCam;

    private double disabledTimestampSave;

    private enum AUTO_MODE_SEL {
        ROCKET_REAR_TO_FEED_STATION(0),
        FEED_STATION_TO_ROCKET_REAR(1),
        ROCKET_FRONT_TO_FEED_STATION(2),
        FEED_STATION_TO_ROCKET_FRONT(3);

        private final int arrayPosition;

        private AUTO_MODE_SEL(int arrayPosition){
            this.arrayPosition = arrayPosition;
        }

        private int getArrayPosition(){
            return arrayPosition;
        }
    }

    public Robot() {
        super(0.2);
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
       // mDrive.zeroSensors(); subsystem manager does this
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            final double kVoltageThreshold = 0.15;
            if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
                //mLED.setLEDOn();
            } else {
                //mLED.setLEDOff();
            }

       /*     try{
                if(visionCam == null){
                    visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
                    System.out.println("RobotInit - VISION CAM IS kUSB1");
                }
            }
            catch(Throwable t){
                System.out.println(t.toString());
            }
    */
            String tractorGain = SmartDashboard.getString("TractorGain", "1.0");

            autoCode = SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode); // or R
     //       autoModesToExecute = determineAutoModesToExecute(autoCode);
    
            greenLEDRingLight = new DigitalOutput(0);
            greenLEDRingLight.set(false); // turn off the light until teleop

            //leftRightCameraControl = new DigitalOutput(5);


            arduinoLed0 = new DigitalOutput(Constants.kArduinoLed0);
            arduinoLed1 = new DigitalOutput(Constants.kArduinoLed1);
            arduinoLed2 = new DigitalOutput(Constants.kArduinoLed2);


            mSubsystemManager.registerEnabledLoops(mEnabledLooper);

            mEnabledLooper.register(mRobotStateEstimator);
            mEnabledLooper.register(mVisionCamProcessor); 
            

            //http://roborio-1731-frc.local:1181/?action=stream
            //   /CameraPublisher/<camera name>/streams=["mjpeg:http://roborio-1731-frc.local:1181/?action=stream", "mjpeg:http://10.17.31.2:1181/?action=stream"]
            try {
                cameraFront = CameraServer.getInstance().startAutomaticCapture(0);
                //cameraBack = CameraServer.getInstance().startAutomaticCapture(1);
                videoSink = CameraServer.getInstance().getServer();
                //selectedCamera = cameraFront;
            }
            catch (Throwable t) {
                System.out.println("Exception while connecting driver camera: " + t.toString());
            }
            SmartDashboard.putString(AUTO_CODE, Constants.kDefaultAutoMode);

            SmartDashboard.putString("TractorGain", "1.2");   
            
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        //zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     * 
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
            mSuperstructure.setOverrideCompressor(true);

            mAutoModeExecuter = null;

            mDrive.zeroSensors();
            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            mLoopersAreRunning = true;
            mSuperstructure.reloadConstants();
            mSuperstructure.openBeak();
            mSuperstructure.uproller();
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
                                                       // L or R
    private AutoModeBase[] determineAutoModesToExecute(String autoCode) {
        //System.out.println("Got this string from the dashboard: " + autoCode);
        AutoModeBase[] autoModes = new AutoModeBase[4];
        //default is LEFT side rocket:
        //default is LEFT side rocket:
        //default is LEFT side rocket:
        autoModes[AUTO_MODE_SEL.ROCKET_REAR_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("1"); // ROCKET REAR TO FEED STATION
        autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_REAR.getArrayPosition()] = AUTO_MODES.get("J"); // FEED STATION TO ROCKET REAR
        autoModes[AUTO_MODE_SEL.ROCKET_FRONT_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("5"); // ROCKET FRONT TO FEED STATION
        autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_FRONT.getArrayPosition()] = AUTO_MODES.get("C"); // FEED STATION TO ROCKET FRONT
        if(autoCode != null && autoCode.length() > 0){
            String autoCodeChar = autoCode.toUpperCase().substring(0,1);
            if("R".equals(autoCodeChar)){
                //RIGHT side rocket:
                //RIGHT side rocket:
                //RIGHT side rocket:
                autoModes[AUTO_MODE_SEL.ROCKET_REAR_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("1"); // ROCKET REAR TO FEED STATION
                autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_REAR.getArrayPosition()] = AUTO_MODES.get("J"); // FEED STATION TO ROCKET REAR
                autoModes[AUTO_MODE_SEL.ROCKET_FRONT_TO_FEED_STATION.getArrayPosition()] = AUTO_MODES.get("5"); // ROCKET FRONT TO FEED STATION
                autoModes[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_FRONT.getArrayPosition()] = AUTO_MODES.get("C"); // FEED STATION TO ROCKET FRONT
            }
        }
        //System.out.println("running auto modes: " + Arrays.toString(autoModes));
		return autoModes;
	}

	private static void initAutoModes() {
        AUTO_MODES = new HashMap<String, AutoModeBase>(); //THESE ARE FROM MARK'S "BIBLE"
        
        AUTO_MODES.put("LeftRocketRearToFeedStationMode",  new LeftRocketRearToFeedStationMode());

        AUTO_MODES.put("1",  new Mode_1());
        AUTO_MODES.put("2",  new Mode_2());
        AUTO_MODES.put("3",  new Mode_3());
        AUTO_MODES.put("4",  new Mode_4());
        AUTO_MODES.put("5",  new Mode_5());
        AUTO_MODES.put("6",  new Mode_6());
        AUTO_MODES.put("7",  new Mode_7());
        AUTO_MODES.put("8",  new Mode_8());
        AUTO_MODES.put("9",  new Mode_9());
        AUTO_MODES.put("10", new Mode_10());
        AUTO_MODES.put("A",  new Mode_A());
        AUTO_MODES.put("B",  new Mode_B());
        AUTO_MODES.put("C",  new Mode_C());
        AUTO_MODES.put("D",  new Mode_D());
        AUTO_MODES.put("E",  new Mode_E());
        AUTO_MODES.put("F",  new Mode_F());
        AUTO_MODES.put("G",  new Mode_G());
        AUTO_MODES.put("H",  new Mode_H());
        AUTO_MODES.put("I",  new Mode_I());
        AUTO_MODES.put("J",  new Mode_J());
        
    }
	
	/**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        periodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mClimber.resetLift();

            // IF TELEOP DOESN"T WORK PUT THESE LINES BACK IN that are shifted to right and commented out below!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // Start loopers
            if (!mLoopersAreRunning) {
                           mEnabledLooper.start();
                           mLoopersAreRunning = true;
                           mDrive.setOpenLoop(DriveSignal.NEUTRAL);
                           mDrive.setBrakeMode(false);
            // Shift to high
                           mDrive.setHighGear(true);
            }
            //zeroAllSensors();
            mSuperstructure.reloadConstants();
            mSuperstructure.setOverrideCompressor(false);
            arduinoLedOutput(Constants.kArduino_TEAM);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     * 
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     * 
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        periodic();
    }

    public void periodic() {
        try {
            greenLEDRingLight.set(true); // turn on the light during teleop
            
            //arduino.setcolor(mVisionCamProcessor.getVisionCamHasTarget() ? GREEN : RED);
            //SmartDashboard.putBoolean("visionCamHasTarget", mVisionCamProcessor.getVisionCamHasTarget());

            double timestamp = Timer.getFPGATimestamp();
            boolean inTractorBeam = false;
            boolean grabCube = mControlBoard.getGrabCubeButton();
            boolean calibrateDown = mControlBoard.getCalibrateDown();
            boolean calibrateUp = mControlBoard.getCalibrateUp();
            boolean spitting = mControlBoard.getSpit();
            boolean pickUp = mControlBoard.getAutoPickUp();
            boolean pickupHatch = mControlBoard.getPickupPanel();
            boolean ejectHatch = mControlBoard.getShootPanel();
            boolean pickupCargo = mControlBoard.getPickupBall();
            boolean ejectCargo = mControlBoard.getShootBall();
            boolean elevCargoShipPos = mControlBoard.getCargoShipBall();
            boolean startingConfiguration = mControlBoard.getStartingConfiguration();       
            int climber = mControlBoard.getClimber();           
           // boolean tractorDrive = mControlBoard.getTractorDrive();

            double elevatorPOV = mControlBoard.getElevatorControl();

            if (elevatorPOV != -1) {
                if (elevatorPOV == 0) {
                    mSuperstructure.setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_FLOOR);
                } else if (elevatorPOV == 1) {
                    mSuperstructure.setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_2ND);
                } else if (elevatorPOV == 2) {
                    mSuperstructure.setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_3RD);
                }
            } else if (elevCargoShipPos) {
                mSuperstructure.setWantedElevatorPosition(ELEVATOR_POSITION.ELEVATOR_SHIP);
            }

            if (climber == 1) {
                mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBINGUP);
            } else if (grabCube) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.INTAKING);
            } else if (spitting) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.SPITTING);
            } else if (calibrateDown) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CALIBRATINGDOWN);
            } else if (calibrateUp) {
            	mSuperstructure.setWantedState(Superstructure.WantedState.CALIBRATINGUP);
            } else if (startingConfiguration){
                mSuperstructure.setWantedState(Superstructure.WantedState.STARTINGCONFIGURATION);
            } else if (pickUp) {
                mSuperstructure.setWantedState(Superstructure.WantedState.AUTOINTAKING);
            } else if (ejectHatch) {
                mSuperstructure.setWantedState(Superstructure.WantedState.EJECTING_HATCH);
            } else if (pickupHatch) {
                mSuperstructure.setWantedState(Superstructure.WantedState.HATCH_CAPTURED); 
            } else if (ejectCargo) {
                mSuperstructure.setWantedState(Superstructure.WantedState.EJECTING_CARGO);
            } else if (pickupCargo) {
                mSuperstructure.setWantedState(Superstructure.WantedState.CARGO_CAPTURED);
            } else {
            	mSuperstructure.setWantedState(Superstructure.WantedState.ELEVATOR_TRACKING);
            }
            	
            // Drive base
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            
            if(mControlBoard.getInvertDrive()){
                joystickAxesAreReversed = !joystickAxesAreReversed;
                //toggleCamera(); 
            }

            if(mControlBoard.getToggleDriveSpeed()){
                driveSpeedIsToggled = !driveSpeedIsToggled;
                //toggleCamera(); 
            }

            if(joystickAxesAreReversed){
                throttle=-throttle;
                //leftRightCameraControl.set(true);
            }

            else{     
                //leftRightCameraControl.set(false);
            }
        
            //if(getInvertCamera()){
            //    toggleCamera(); 
            //}
            //videoSink.setSource(selectedCamera);

            Optional<ShooterAimingParameters> aimParams;
            double now = Timer.getFPGATimestamp();
            aimParams = mRobotState.getAimingParameters();
            mDrive.setAimingParams(aimParams);

            if (mControlBoard.getTractorDrivePickupHatch() || mControlBoard.getTractorDriveEjectHatch())    
            { 

                if (mControlBoard.getTractorDrivePickupHatch()) {

                    if (!mTractorBeamPickupSelected) { // this is the first time in
                        mSuperstructure.prepareToPickupHatch();
                        mTractorBeamPickupSelected = true;
                    }

                    if  (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  ) { 
                    mDrive.setWantTractorBeam();
                    }

                    if (mDrive.isTBFinished()) {
                        mSuperstructure.openBeak();
                    }
                }

                if (mControlBoard.getTractorDriveEjectHatch()) {
                    // System.out.println("im here!!!!!!!");
                    
                    if  (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  ) {
                        mDrive.setWantTractorBeam();
                        }

                    if (mDrive.isTBFinished()) {
                     //+   mSuperstructure.ejectHatch();
                    }
                }
              //  System.out.println("A");
                if ((mDrive.isFirstTimeInTractorBeam()) &&   (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5  )) { 
               //     System.out.println("B");
                    mDrive.setIsDrivingTractorBeam(true);
                }
            } else if (!mControlBoard.getAutoLevel1ToCargoL1() &&           //5
                         !mControlBoard.getAutoCargoL1ToFeederStation() &&  //6
                         !mControlBoard.getAutoFeederStationToFront() &&    //4
                         !mControlBoard.getAutoFeederStationToRear() &&     //2
                         !mControlBoard.getAutoFrontToFeederStation()&&     //3
                         !mControlBoard.getAutoRearToFeederStation() &&     //1
                         !mControlBoard.getAutoLevel1ToRear()) {            //7
                mDrive.resetTractorBeam();
                if (mTractorBeamPickupSelected) {
                    mTractorBeamPickupSelected = false;
                    mSuperstructure.openBeak();

                }
            }

            if(mControlBoard.getAutoRearToFeederStation()){ //1
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftRocketRearToFeedStationMode );
                    mAutoModeExecuter.start();    
                }
            }
            else if(mControlBoard.getAutoFeederStationToRear()){ //2
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftFeedStationToRocketRearMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoFrontToFeederStation()){ //3
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftRocketFrontToFeedStationMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoFeederStationToFront()){ //4
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftFeedStationToRocketFrontMode );
                    mAutoModeExecuter.start();
                }
            }
            else if(mControlBoard.getAutoLevel1ToCargoL1()){ //5
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftLevel1ToCargoL1Mode );
                    mAutoModeExecuter.start();
                }
            }          
             else if(mControlBoard.getAutoCargoL1ToFeederStation()){ //6
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftCargoL1ToFeederStationMode );
                    mAutoModeExecuter.start();
                }
            }

            else if(mControlBoard.getAutoLevel1ToRear()){  //7
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( newLeftLevel1ToRocketRearMode );
                    mAutoModeExecuter.start();
                }
            }


            /*
            else if(mControlBoard.getTractorDrive()){
                if(mAutoModeExecuter == null){
                    mAutoModeExecuter = new AutoModeExecuter();
                    mAutoModeExecuter.setAutoMode( new PlacePanel());
                    //mAutoModeExecuter.setAutoMode(autoModesToExecute[AUTO_MODE_SEL.FEED_STATION_TO_ROCKET_FRONT.getArrayPosition()]);
                    mAutoModeExecuter.start();
                }
            }
            */

            else if((climber != 1) && !mDrive.isDrivingTractorBeam()){
                stopAuto(); // if none of the above 4 auto buttons is being held down and we're not climbing

                // if(tractorDrive && mVisionCamProcessor.getVisionCamHasTarget()){
                //     try {
                //         String tractorGain = SmartDashboard.getString("TractorGain", "1.0");
                //         mTractorBeamGain = Double.parseDouble(tractorGain);
                //         turn = mTractorBeamGain*((mVisionCamProcessor.getVisionCamXPosition()-160)/160.0);
                //         SmartDashboard.putNumber("VisionTurnValue", turn);
                //     } catch(NumberFormatException e){
                //         SmartDashboard.putString("VisionStatusRobot.java", "An exception ocurred...");
                //         System.out.println(e.toString());
                //     }
                //     arduinoLedOutput(Constants.kArduino_BLUE);    
                //     tractorIndicator = Boolean.TRUE;
                // } else {
                //     arduinoLedOutput(Constants.kArduino_RED);
                //     tractorIndicator = Boolean.FALSE;
                // }

                //regular cheesy drive
                if(driveSpeedIsToggled){
                    throttle *= 0.5;//'slow' driver controller speed
                    turn *= 0.3;
                    arduinoLedOutput(Constants.kArduino_YELLW);
                }
                else{
                    throttle *= 0.9;//'normal' driver controller speed
                    turn *= 0.5;
                    arduinoLedOutput(Constants.kArduino_BLUEW);
                }
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                                                                    !mControlBoard.getLowGear()));
                boolean wantLowGear = mControlBoard.getLowGear();
                mDrive.setHighGear(!wantLowGear);
                mClimber.setWantedState(Climber.WantedState.IDLE);

                if(mRobotState.getFieldToVehicle(timestamp).getTranslation().x() > (Constants.FIELD_CTR_INCHES - Constants.WARNING_BUFFER_INCHES)){
               //     mControlBoard.rumbleDriver();
                }
            }
            //else{
            //    stopAuto(); // if none of the above 4 auto buttons is being held down and we're climbing
                            // NOTE: the superstructure controls the drive wheels during a climb
            //}

            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /*
     * public boolean getInvertCamera(){ boolean invertCamera=false;
     * synchronized(invertCameraPrevious){ boolean invertCameraCurrent =
     * (selectedCamera == cameraFront && mControlBoard.getBackCamera()) ||
     * (selectedCamera == cameraBack && mControlBoard.getFrontCamera());
     * if(invertCameraCurrent && !invertCameraPrevious){ invertCamera=true; }
     * invertCameraPrevious = invertCameraCurrent; } return invertCamera; }
     * 
     * private void toggleCamera(){ camerasAreReversed = !camerasAreReversed; if
     * (selectedCamera == cameraFront) { selectedCamera = cameraBack;
     * arduinoLedOutput(Constants.kArduino_BLUEW); } else { selectedCamera =
     * cameraFront; arduinoLedOutput(Constants.kArduino_BLUEW); } }
     */
    private void arduinoLedOutput(int value) {
        arduinoLed0.set((value & 0x01)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed1.set((value & 0x02)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed2.set((value & 0x04)==0 ? Boolean.FALSE: Boolean.TRUE);
    }

    private void stopAuto(){
        if(mAutoModeExecuter != null){
            mAutoModeExecuter.stop();
            mAutoModeExecuter = null;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

    //        if (mAutoModeExecuter != null) {
    //            mAutoModeExecuter.stop();
    //        }
    //        mAutoModeExecuter = null;

      //      mEnabledLooper.stop();

            // Call stop on all our Subsystems.
      //      mSubsystemManager.stop();

      //      mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            // If are tuning, dump map so far.
            if (Constants.kIsShooterTuning) {
                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
                    System.out.println("{" +
                            entry.getKey().value + ", " + entry.getValue().value + "},");
                }
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        final double kVoltageThreshold = 0.15;
        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
            //mLED.setLEDOn();
        } else {
            //mLED.setLEDOff();
        }

        autoCode = SmartDashboard.getString("AutoCode", autoCode); // or R
        Optional<ShooterAimingParameters> aimParams;
        double now = Timer.getFPGATimestamp();
        aimParams = mRobotState.getAimingParameters();
        mDrive.setAimingParams(aimParams);
        
        greenLEDRingLight.set(false); // turn off the light until teleop
        allPeriodic();
        /*
        if(visionCam == null){
            //visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
            //System.out.println("VISION CAM IS kUSB1");
        }
        */


        // double disabledTimestamp = Timer.getFPGATimestamp();
        // if((disabledTimestamp - disabledTimestampSave) > 2){
        //     zeroAllSensors();
        //     disabledTimestampSave = disabledTimestamp;
        // }
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = Elevator.getInstance().checkSystem();
        results &= Drive.getInstance().checkSystem();
        results &= Intake.getInstance().checkSystem();


        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
         mRobotState.outputToSmartDashboard();
         mSubsystemManager.outputToSmartDashboard();
         mSubsystemManager.writeToLog();
          mEnabledLooper.outputToSmartDashboard();
        SmartDashboard.putString("AutoCode", autoCode);
        SmartDashboard.putBoolean("Tractor Beam", tractorIndicator);
        // SmartDashboard.putString("AutoCodesReceived", autoCodes);
        // //SmartDashboard.putString("SerialPorts", Arrays.toString(SerialPort.Port.values()));
        // SmartDashboard.putBoolean("Cal Dn", mControlBoard.getCalibrateDown());
        // SmartDashboard.putBoolean("Cal Up", mControlBoard.getCalibrateUp());
        // SmartDashboard.putBoolean("TapeSensor", tapeSensor.get());
        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
        //AutoSelectorSanityCheck();
        //UpdateAutoDriving();
    }
}
