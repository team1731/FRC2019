package org.usfirst.frc.team1731.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1731.lib.util.CheesyDriveHelper;
import org.usfirst.frc.team1731.lib.util.CrashTracker;
import org.usfirst.frc.team1731.lib.util.DelayedBoolean;
import org.usfirst.frc.team1731.lib.util.DriveSignal;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.InterpolatingTreeMap;
import org.usfirst.frc.team1731.lib.util.LatchedBoolean;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.robot.Constants.ELEVATOR_POSITION;
import org.usfirst.frc.team1731.robot.Constants.GRABBER_POSITION;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeExecuter;
import org.usfirst.frc.team1731.robot.auto.modes.StandStillMode;
import org.usfirst.frc.team1731.robot.auto.modes.TestAuto;
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
import org.usfirst.frc.team1731.robot.loops.VisionProcessor;
import org.usfirst.frc.team1731.robot.paths.DriveForward;
import org.usfirst.frc.team1731.robot.paths.profiles.PathAdapter;
import org.usfirst.frc.team1731.robot.subsystems.ConnectionMonitor;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Elevator;

import org.usfirst.frc.team1731.robot.subsystems.Intake;
//import org.usfirst.frc.team1731.robot.subsystems.LED;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import org.usfirst.frc.team1731.robot.subsystems.Wrist;
import org.usfirst.frc.team1731.robot.subsystems.Wrist.WristPositions;
import org.usfirst.frc.team1731.robot.subsystems.Climber;
import org.usfirst.frc.team1731.robot.vision.VisionServer;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer; //edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SerialPort;

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
public class Robot extends IterativeRobot {
    private DigitalOutput leftRightCameraControl;
		
	private static final String AUTO_CODES = "AutoCodes";
    private static Map<String, AutoModeBase> AUTO_MODES; // modes defined in Mark's "BIBLE"
    
    
	static {
		initAutoModes();
	}
	
	// Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    private Climber mClimber = Climber.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    //private LED mLED = LED.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;

    private AutoModeBase[] autoModesToExecute;

    private boolean joystickAxesAreReversed;
    private boolean camerasAreReversed;
    
    private UsbCamera cameraFront;
    private UsbCamera cameraBack;
    private UsbCamera selectedCamera;
    private DigitalOutput arduinoLed0;
    private DigitalOutput arduinoLed1;
    private DigitalOutput arduinoLed2;

    private Boolean invertCameraPrevious = Boolean.FALSE;
    private NetworkTable networkTable;
    private VideoSink videoSink;

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
                            Arrays.asList(Drive.getInstance(), Superstructure.getInstance(),
                                    Elevator.getInstance(), Intake.getInstance(), Climber.getInstance(),
                                    ConnectionMonitor.getInstance()/*, LED.getInstance() , Wrist.getInstance()*/ ));

    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    //private VisionServer mVisionServer = VisionServer.getInstance();

    private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);

    //private DelayedBoolean mDelayedAimButton;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    //private static Solenoid _24vSolenoid = Constants.makeSolenoidForId(11, 2);
    
    private DigitalInput tapeSensor;
 
    private SerialPort visionCam;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            try{
                visionCam = new SerialPort(115200, SerialPort.Port.kUSB2); // .kOnBoard .kUSB2
            }
            catch(Throwable t){
                System.out.println(t.toString());
            }
        
            String autoCodes = SmartDashboard.getString("AutoCodes", "2A");
            autoModesToExecute = determineAutoModesToExecute(autoCodes);
    
            networkTable = NetworkTable.getTable("");

            leftRightCameraControl = new DigitalOutput(5);

            tapeSensor = new DigitalInput(0);
            arduinoLed0 = new DigitalOutput(Constants.kArduinoLed0);
            arduinoLed1 = new DigitalOutput(Constants.kArduinoLed1);
            arduinoLed2 = new DigitalOutput(Constants.kArduinoLed2);
            SmartDashboard.putBoolean("TapeSensor", tapeSensor.get());

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
          //  mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            //mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
            
            //http://roborio-1731-frc.local:1181/?action=stream
            //   /CameraPublisher/<camera name>/streams=["mjpeg:http://roborio-1731-frc.local:1181/?action=stream", "mjpeg:http://10.17.31.2:1181/?action=stream"]
            
            cameraFront = CameraServer.getInstance().startAutomaticCapture(0);
            cameraBack = CameraServer.getInstance().startAutomaticCapture(1);
            videoSink = CameraServer.getInstance().getServer();
            selectedCamera = cameraFront;

           	SmartDashboard.putString(AUTO_CODES, "2A");
            
          //  mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
            // Force an true update now to prevent robot from running at start.
          //  mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
            //PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
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

            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            mSuperstructure.reloadConstants();
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
                                                       // 10B for example
    private AutoModeBase[] determineAutoModesToExecute(String autoCodes) {
        System.out.println("Got this string from the dashboard: " + autoCodes);
        AutoModeBase[] autoModes = new AutoModeBase[2]; 
        if(autoCodes != null && autoCodes.length() >=2 && autoCodes.length() <= 3){
            AutoModeBase selectedAutoMode1 = null;
            AutoModeBase selectedAutoMode2 = null;
            if(autoCodes.length() == 2){
                selectedAutoMode1 = AUTO_MODES.get(autoCodes.substring(0, 1));
                selectedAutoMode2 = AUTO_MODES.get(autoCodes.substring(1, 1));
            }
            else{
                selectedAutoMode1 = AUTO_MODES.get(autoCodes.substring(0, 2));
                selectedAutoMode2 = AUTO_MODES.get(autoCodes.substring(2, 3));
            }
            autoModes[0] = selectedAutoMode1;
            autoModes[1] = selectedAutoMode2;
        }
        System.out.println("running auto modes: " + Arrays.toString(autoModes));
		return autoModes;
	}

	private static void initAutoModes() {
    	AUTO_MODES = new HashMap<String, AutoModeBase>(); //THESE ARE FROM MARK'S "BIBLE"
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
        allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            // Shift to high
            mDrive.setHighGear(true);
            zeroAllSensors();
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
        try {
            
            double timestamp = Timer.getFPGATimestamp();

            boolean overTheTop = mControlBoard.getOverTheTopButton();
            boolean flipUp = mControlBoard.getFlipUpButton();
            boolean flipDown = mControlBoard.getFlipDownButton();
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
            boolean tracktorDrive = mControlBoard.getTractorDrive();          

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
            	
            if (flipUp) {
                mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_UP);
            } else if (flipDown) {
                mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_DOWN);
            } else {
                mSuperstructure.setOverTheTop(GRABBER_POSITION.FLIP_NONE);
            }


            // Drive base
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();
            
            if(mControlBoard.getInvertDrive()){
                joystickAxesAreReversed = !joystickAxesAreReversed;
                toggleCamera(); 
            }

            if(joystickAxesAreReversed){
                throttle=-throttle;
                leftRightCameraControl.set(true);
            }

            else{     
                leftRightCameraControl.set(false);
            }
        
            if(getInvertCamera()){
                toggleCamera(); 
            }
            videoSink.setSource(selectedCamera);
            
            if(tracktorDrive && visionCam != null) {
                String[] visionTargetPositions = visionCam.readString().split(",");
                for(String visionTargetPosition : visionTargetPositions){
                    if(visionTargetPosition.length() > 0){
                        System.out.println(visionTargetPosition);                        
                        try{
                            turn = (Double.valueOf(visionTargetPosition)-160)/160;
                            System.out.println("TURN: " + turn);
                        }
                        catch(NumberFormatException e){
                            System.out.println(e.toString());
                        }
                        arduinoLedOutput(Constants.kArduino_GREEN);
                    }
                    else {
                        System.out.println("No data received from vision camera");
                        arduinoLedOutput(Constants.kArduino_RED);
                    }
                } 
            }

            if(climber != 1){
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                        !mControlBoard.getLowGear()));
                boolean wantLowGear = mControlBoard.getLowGear();
                mDrive.setHighGear(!wantLowGear);
                mClimber.setWantedState(Climber.WantedState.IDLE);
            }

            if(mControlBoard.getTestWrist()){
                Wrist.getInstance().setWantedPosition(WristPositions.STRAIGHTAHEAD);
                //mSuperstructure.setWantedState(Superstructure.WantedState.WRIST_TRACKING);
                Wrist.getInstance().setWantedState(Wrist.WantedState.WRISTTRACKING);
            }
            
             // Handle ball pickup and shooting
             if(mControlBoard.getPickupBall() && !mControlBoard.getShootBall()){

            }
            else if(mControlBoard.getShootBall() && !mControlBoard.getPickupBall()){
                if(Elevator.getInstance().atDesired()){
                    
                }
             }



            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public boolean getInvertCamera(){
        boolean invertCamera=false;
        synchronized(invertCameraPrevious){
          boolean invertCameraCurrent = (selectedCamera == cameraFront && mControlBoard.getBackCamera()) ||
                                        (selectedCamera == cameraBack && mControlBoard.getFrontCamera());
            if(invertCameraCurrent && !invertCameraPrevious){
                invertCamera=true;
            }
            invertCameraPrevious = invertCameraCurrent;
        }
        return invertCamera;
    }

    private void toggleCamera(){
        camerasAreReversed = !camerasAreReversed;
        if (selectedCamera == cameraFront) {
            selectedCamera = cameraBack;
            arduinoLedOutput(Constants.kArduino_BLUEW);
        } else {
            selectedCamera = cameraFront;
            arduinoLedOutput(Constants.kArduino_BLUEW);
        } 
      }

    private void arduinoLedOutput(int value) {
        arduinoLed0.set((value & 0x01)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed1.set((value & 0x02)==0 ? Boolean.FALSE: Boolean.TRUE);
        arduinoLed2.set((value & 0x04)==0 ? Boolean.FALSE: Boolean.TRUE);
    }

    private void AutoSelectorSanityCheck(){
        String received = SmartDashboard.getString("Auto Selector", "1").toUpperCase();
        if(received != "1" && received != "2" && received != "3" && received != "4" && received != "5" && received != "6" && received != "7" && received != "8" && received != "9" && received != "10"
        && received != "A" && received != "B" && received != "C" && received != "D" && received != "E" && received != "F" && received != "G" && received != "H" && received != "I" && received != "J"){
            SmartDashboard.putString("Auto Selector", "1");
        }
    }

    private AutoModeBase StringToAutoMode(String input){
        switch(input){
            case "1":
                return new Mode_1();
            case "2":
                return new Mode_2();
            case "3":
                return new Mode_3();
            case "4":
                return new Mode_4();
            case "5":
                return new Mode_5();
            case "6":
                return new Mode_6();
            case "7":
                return new Mode_7();
            case "8":
                return new Mode_8();
            case "9":
                return new Mode_9();
            case "10":
                return new Mode_10();
            case "A":
                return new Mode_A();
            case "B":
                return new Mode_B();
            case "C":
                return new Mode_C();
            case "D":
                return new Mode_D();
            case "E":
                return new Mode_E();
            case "F":
                return new Mode_F();
            case "G":
                return new Mode_G();
            case "H":
                return new Mode_H();
            case "I":
                return new Mode_I();
            case "J":
                return new Mode_J();           
            default:
                return new StandStillMode();
        }
    }

    private void UpdateAutoDriving(){
        if(mControlBoard.getActivateAuto()){
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(StringToAutoMode(SmartDashboard.getString("Auto Selector", "1")));
            mAutoModeExecuter.start();
        } else if(mControlBoard.getDeactivateAuto()){
            if(mAutoModeExecuter != null){
                mAutoModeExecuter.stop();
            }
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();

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

        try{
            if(visionCam == null){
                visionCam = new SerialPort(115200, SerialPort.Port.kUSB);
                System.out.println("VISION CAM IS kUSB");
            }
            if(visionCam == null){
                visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
                System.out.println("VISION CAM IS kUSB1");
            }
            if(visionCam == null){
                visionCam = new SerialPort(115200, SerialPort.Port.kUSB2);
                System.out.println("VISION CAM IS kUSB2");
            }
        }
        catch(Throwable t){
            System.out.println(t.toString());
        }

        zeroAllSensors();
        allPeriodic();
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

        SmartDashboard.putString("AutoCodesReceived", autoCodes);
        SmartDashboard.putString("SerialPorts", Arrays.toString(SerialPort.Port.values()));
        SmartDashboard.putBoolean("Cal Dn", mControlBoard.getCalibrateDown());
        SmartDashboard.putBoolean("Cal Up", mControlBoard.getCalibrateUp());
        SmartDashboard.putBoolean("TapeSensor", tapeSensor.get());
        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
        AutoSelectorSanityCheck();
        UpdateAutoDriving();
    }
}
