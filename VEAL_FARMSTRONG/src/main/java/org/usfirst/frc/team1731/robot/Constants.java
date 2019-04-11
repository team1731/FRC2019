package org.usfirst.frc.team1731.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import org.usfirst.frc.team1731.lib.util.ConstantsBase;
import org.usfirst.frc.team1731.lib.util.InterpolatingDouble;
import org.usfirst.frc.team1731.lib.util.InterpolatingTreeMap;
import org.usfirst.frc.team1731.lib.util.math.PolynomialRegression;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants extends ConstantsBase {

    public static String kDefaultAutoMode = "L";
    
    public static double kLooperDt = 0.005;

    // Target parameters
    // Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
    // Section 3.13
    // ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
    // Parts GE-17203-FLAT and GE-17371 (sheet 7)
    public static double kBoilerTargetTopHeight = 27.5; //was 19.5
    public static double kBoilerRadius = 0; //was 7.5

    public static int FIELD_CTR_INCHES = 27 * 12;
    public static int WARNING_BUFFER_INCHES = 36;

    // Wrist parameters
   // public static int kWristTalon = 4;
    public static int kWristKI = 0;
    public static int kWristKD = 0;
    public static int kWristTalonKF = 0;
    public static int kWristTalonKP = 30;

    
    // Shooter tuning parameters
    public static boolean kIsShooterTuning = false;
    public static double kShooterTuningRpmFloor = 2900;
    public static double kShooterTuningRpmCeiling = 3500;
    public static double kShooterTuningRpmStep = 50;
    public static double kShooterTuningRpm = 3500.0;

    /* ROBOT PHYSICAL CONSTANTS */
    // encoder has 4096 /3 tics per revolution
    // Wheels  was 3.4149
    public static double kDriveWheelDiameterInches = 8;  //factor of 3 in
    public static double kTrackWidthInches = 22.00;
    public static double kTrackScrubFactor = 0.924;

    // Geometry
    public static double kCenterToFrontBumperDistance = 19.25; //16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 19.25; //16.99;
    public static double kCenterToSideBumperDistance = 16.75; //17.225;

    // Shooting suggestions
    public static double kOnTargetErrorThreshold = 3.0;

    // Intake Voltages
    public static double kIntakeVoltageMax = 7.5;
    public static double kIntakeVoltageMin = 5.5;
    public static double kIntakeShootingVoltage = 4.0;
    public static final double kIntakeVoltageDifference = kIntakeVoltageMax - kIntakeVoltageMin;

    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 1.8;   //cheezy guys had 1.2
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 6;    //cheezy guys had 6.0
    public static double kDriveHighGearVelocityKf = 0.39;    //cheezy guys had .15. Orig: 0.7
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 0.05;// 240.0;
    public static double kDriveHighGearNominalOutput = 0.0;   // cheezy gusy had .5
    public static double kDriveHighGearMaxSetpoint = 150; //17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 2;  // was 1.0
    public static double kDriveLowGearPositionKi = 0;   // was 0
    public static double kDriveLowGearPositionKd = 30.0; // was 100
    public static double kDriveLowGearPositionKf = 0.39; // was 2.62 ---- 0.(9)7 * 2.7 gear ratio. Orig: 1.91
    public static int kDriveLowGearPositionIZone = 0; // was 0
    public static double kDriveLowGearPositionRampRate = 0.05; // was 0.05//240.0; // V/s
    public static double kDriveLowGearNominalOutput = 0.05; // was 0.05// V
    
    //TODO need to set these velocities and accelerations  This is for the talon motion magic so I think you need to do the tics per 100ms thing
   // public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps                                                                                                           // in RPM
    // 400 ticks per 100mS (400 * 1.26 is how many units of 1023 power units)
    //
    // 34 tooth 3/8     36 tooth dog     52 tooth dog    18 tooth 3/8
    // 6.0:1
    // 16.37:1
    // 
    public static int kDriveLowGearMaxVelocity = 1900;  //these are units of units per 100ms - bdl - this number works out to 3.92 ft/sec
    
    //public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
    public static int kDriveLowGearMaxAccel = 1900;  //These are   in units per 100ms                                                                                                       // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;

    public static double kTBMaxSpeed = 90;
    public static double kTBGain = 3.0;
    public static double kTBWallDistance = 2000;  //This is what the IR sensor reads when you hit the wall

    // Turn to heading gains
    public static double kDriveTurnKp = 3.0;
    public static double kDriveTurnKi = 1.5;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.0;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 360.0;
    public static double kDriveTurnMaxAcc = 720.0;

    // Shooter gains
    public static double kShooterTalonKP = 0.16;
    public static double kShooterTalonKI = 0.00008;
    public static double kShooterTalonKD = 0.0;
    public static double kShooterTalonKF = 0.035;
    public static double kShooterRampRate = 60.0;

    public static double kShooterTalonHoldKP = 0.0;
    public static double kShooterTalonHoldKI = 0.0;
    public static double kShooterTalonHoldKD = 0.0;

    public static double kShooterHoldRampRate = 720.0;

    public static int kShooterTalonIZone = 1000;// 73 rpm
    public static int kShooterOpenLoopCurrentLimit = 35;

    public static double kShooterSetpointDeadbandRpm = 1.0;

    // Used to determine when to switch to hold profile.
    public static double kShooterMinTrackStability = 0.25;
    public static double kShooterStartOnTargetRpm = 50.0;
    public static double kShooterStopOnTargetRpm = 150.0;
    public static int kShooterKfBufferSize = 20;
    public static int kShooterMinOnTargetSamples = 20; // Should be <= kShooterKvBufferSize

    public static int kShooterJamBufferSize = 30;
    public static double kShooterDisturbanceThreshold = 25;
    public static double kShooterJamTimeout = 1.5; // In secs
    public static double kShooterUnjamDuration = 0.5; // In secs
    public static double kShooterMinShootingTime = 1.0; // In secs

    public static double kShooterSpinDownTime = 0.25;

    // Feeder gains
    public static double kFeederKP = 0.02;
    public static double kFeederKI = 0.0;
    public static double kFeederKD = 0.2;
    public static double kFeederKF = 0.009;
    public static double kFeederRampRate = 240.0;
    public static double kFeederVoltageCompensationRampRate = 10.0;
    public static double kFeederFeedSpeedRpm = 5400.0;
    public static double kFeederSensorGearReduction = 3.0;
    
   //Elevator
    public static enum GRABBER_POSITION {
    	FLIP_DOWN,
    	FLIP_NONE,
    	FLIP_UP,
    	FLIP_UN_INIT
    }

    //Elevator
    public static final int kElevatorTalon = 7;
    public static double kElevatorTalonKP = 5; //2.0;
    public static double kElevatorTalonKI = 0.0;
    public static double kElevatorTalonKD = 0.0;
    public static double kElevatorTalonKF = 0.0512; //0.0;

    public static enum ELEVATOR_POSITION {
    	ELEVATOR_FLOOR,
    	ELEVATOR_2ND,
        ELEVATOR_3RD,
        ELEVATOR_SHIP,
        ELEVATOR_CARGO_PICKUP
    }
    public static int kElevatorTicsPerInch = 7420;
    public static int kElevatorLevel_Floor = 0;
    public static int kElevatorLevel_2nd = 1;
    public static int kElevatorLevel_3rd = 2;
    public static int kElevatorTopEncoderValue = 600000;
    public static int kElevatorHatch3rd_EncoderValue=462264 - 3*kElevatorTicsPerInch;// 9500; //was 9918
    public static int kElevatorHatch2nd_EncoderValue= 233632 - 2*kElevatorTicsPerInch;//4872;
    public static int kElevatorHatchFloor_EncoderValue= 0; 
    public static int kElevatorHatchShip_EncoderValue=233632; //1200;
    public static int kElevatorCargo3rd_EncoderValue =462264;  //9500; // was 9918
    public static int kElevatorCargo2nd_EncoderValue =304544; //6351;  haymarket had 254496
    public static int kElevatorCargoFloor_EncoderValue =70912 + (int)(1.5*kElevatorTicsPerInch); //1580;
    public static int kElevatorCargoShip_EncoderValue= 233632 - 6*kElevatorTicsPerInch; //make it 6" lower (Haymarket playoffs)
    public static int kElevatorBallPickup_EncoderValue= 0; //was 1044
    public static int kElevatorHomeEncoderValue = 0;
    public static int kElevatorEncoderRange = 20;
    public static int kElevatorCruiseVelocity = 22000;//15000
    public static int kElevatorAcceleration = 30000; //6000

    //public static double kElevatorPeakForward = 0.6;
    //public static double kElevatorPeakReverse = -0.3;
    public static double kElevatorCalibrateUp = 0.8;
    public static double kElevatorCalibrateDown = -0.5;
    
    //Climber
    public static final int kClimberTalonLeft = 9;
    public static final int kClimberTalonRight = 4;

    public static double kClimberTalonKP = 80; //was 6 working
    public static double kClimberTalonKI = 0.0;
    public static double kClimberTalonKD = 0.0;
    public static double kClimberTalonKF = 11; //was .2 working

    public static int kClimberSlowCruiseVelocity = 10; //15000
    public static int kClimberMediumCruiseVelocity = 10; //15000
    public static int kClimberFastCruiseVelocity = 35; //15000
    public static int kClimberAcceleration = 30; //6000
    public static double kClimberExtendPercent = 1.0; //0.8;
    public static double kClimberRetractPercent = -1.0; //-0.6;

   
    public static int kClimberRetractedPositionLeft = 80; //left is 25
    public static int kClimberDartsHaveHitTheFloorLeft = 300;
    public static int kClimberDartsHaveDetachedLeft = 700;
    public static int kClimberExtendedPositionLeft =  kClimberRetractedPositionLeft + 800; // +9.5 inches ( was 670; )
    

    public static int kClimberRetractedPositionRight = 63; //right is 225
    public static int kClimberDartsHaveHittheFloorRight = 300;
    public static int kClimberDartsHaveDetachedRight = 700;
    public static int kClimberExtendedPositionRight = kClimberRetractedPositionRight + 800; // +9.5 inches (was 875;)


    public static int kClimberStartWheelsLeft = 600;
    public static int kClimberStartWheelsRight = 600;



    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

    /* TALONS */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    // Drive
    public static final int kLeftDriveMasterId = 5;
    public static final int kLeftDriveSlaveId = 6;
    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriverSlaveId = 2;
    
    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from
     * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
     * configuration.
     */
    public static final int kSlotIdx = 0;

    /*
     * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    /*
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int kTimeoutMs = 10; // I think this should be more like 10 Brent - was 10 but not working.
    
    /* choose so that Talon does not report sensor out of phase */
    public static boolean kSensorPhase = true;

    /* choose based on what direction you want to be positive,
        this does not affect motor invert. */
    public static boolean kMotorInvert = false;

    /* VICTORS */
    // Feeder
	public static final int kFeederVictor = 1;

    // Intake
	public static final int kIntakeTalon = 8;

    // Shooter
//	public static final int kShooterVictor = 4;

    //
    // PCM 0 SOLENOIDS
    //
    public static final int kBeakSwinger1 = 4; 
    public static final int kBeakSwinger2 = 5; 
    public static final int kMustache1 = 7; 
    public static final int kMustache2 = 6; 
    public static final int kDartLatch1 = 0;
    public static final int kDartLatch2 = 1;

    //
    // PCM 1 SOLENOIDS
    //
    public static final int kBeakOpener1 = 0;
    public static final int kBeakOpener2 = 5;
    
    public static final int kRotateWristShort1 = 2;
    public static final int kRotateWristShort2 = 4;
    
    public static final int kRotateWristLong1 = 3;
    public static final int kRotateWristLong2 = 1;
    
    public static final int kTopRoller1 = 7;
    public static final int kTopRoller2 = 6;

	public static final int SlotIdx = 0;

	public static  double kRotateTime = 500;
    public static  double kUnlockClimberTime = 500;

    // Analog Inputs
    public static int kLEDOnId = 2;

    // Digital Outputs
    public static int kArduinoLed0 = 7;
    public static int kArduinoLed1 = 8;
    public static int kArduinoLed2 = 9;
    public static int kArduino_TEAM  = 0; // blue/yellow 1731 team wheel
    public static int kArduino_RED   = 1; // solid red
    public static int kArduino_GREEN = 2; // solid green
    public static int kArduino_BLUE  = 3; // solid blue
    public static int kArduino_REDW = 4; // red wipe
    public static int kArduino_GREENW = 5; // green wipe
    public static int kArduino_BLUEW = 6; // blue wipe
    public static int kArduino_YELLW = 7; // yellow wipe

    // Phone
    public static int kAndroidAppTcpPort = 8254;

    // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 5.0;   // cheezy people had 5.0
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75; //cheezyguys hae this as .75
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;  // cp had 1.0
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;

    // Pose of the camera frame w.r.t. the robot frame
    public static double kCameraXOffset = 16.0;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 4.0;  // they had 20.9
    public static double kCameraPitchAngleDegrees = 18.135; // Measured on 3/30
    public static double kCameraYawAngleDegrees = 0.0;
    public static double kCameraDeadband = 0.0;

    /* AIM PARAMETERS */

    public static double kDefaultShootingDistanceInches = 95.8;
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = true; // Change to 'true' to use the best-fit polynomial
                                                                // instead.
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double kShooterOptimalRange = 100.0;
    public static double kShooterOptimalRangeFloor = 95.0;
    public static double kShooterOptimalRangeCeiling = 105.0;

    public static double kShooterAbsoluteRangeFloor = 90.0;
    public static double kShooterAbsoluteRangeCeiling = 130.0;

    public static double[][] kFlywheelDistanceRpmValues = {
            // At champs 4/27
            { 90.0, 2890.0 },
            { 95.0, 2940.0 },
            { 100.0, 2990.0 },
            { 105.0, 3025.0 },
            { 110.0, 3075.0 },
            { 115.0, 3125.0 },
            { 120.0, 3175.0 },
            { 125.0, 3225.0 },
            { 130.0, 3275.0 },
    };
  
	public static int kPidIdx = 0;


	



	

    static {
        for (double[] pair : kFlywheelDistanceRpmValues) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        kDefaultShootingRPM = kFlywheelAutoAimMap
                .getInterpolated(new InterpolatingDouble(Constants.kDefaultShootingDistanceInches)).value;

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelDistanceRpmValues, 2);
    }

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
    	System.out.println("creating solenoid id " + solenoidId + " PCM " + solenoidId/8 + " CHAN " + solenoidId%8);
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int pcmChannel, int solenoidId) {
    	System.out.println("creating solenoid id " + solenoidId + " PCM " + pcmChannel + " CHAN " + solenoidId);
        return new Solenoid(pcmChannel, solenoidId);
    }

    public static DoubleSolenoid makeDoubleSolenoidForIds(int pcmChannel, int forward_solenoidId, int reverse_solenoidId) {
    	System.out.println("creating solenoid ids " + forward_solenoidId + "-" + reverse_solenoidId + " PCM " + pcmChannel + " CHAN ");
        return new DoubleSolenoid(pcmChannel, forward_solenoidId, reverse_solenoidId);
    }

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
