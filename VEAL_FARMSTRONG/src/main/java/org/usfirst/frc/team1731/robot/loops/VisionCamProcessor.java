package org.usfirst.frc.team1731.robot.loops;

import java.util.ArrayList;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.vision.TargetInfo;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
@Deprecated
public class VisionCamProcessor implements Loop {
    static VisionCamProcessor instance_;

    public static VisionCamProcessor getInstance() {
        if(instance_ == null){
            instance_ = new VisionCamProcessor();
        }
        return instance_;
    }

    private VisionCamProcessor() {
    }

    private RobotState mRobotState = RobotState.getInstance();
    private SerialPort visionCam;

    private void attemptVisionCamConnection(){
        SmartDashboard.putBoolean("visionCamConnected", false);
        try {
            if(visionCam == null){
                visionCam = new SerialPort(115200, SerialPort.Port.kMXP);
            }
            if(visionCam != null){
                //visionCam.writeString("streamoff\n");
                //visionCam.writeString("usbsd\n");
                visionCamAvailable = true;
                SmartDashboard.putBoolean("visionCamConnected", true);
            }
        } catch(Exception e){
            System.err.println(e.toString());
        }
    }

    public void setVisionCam(SerialPort visionCam){
        this.visionCam = visionCam;
        visionCamAvailable = true;
    }

    //#region Vision Camera Variables
    boolean visionCamAvailable;
    boolean visionCamHasTarget;
    double visionCamZPosition;
    double visionCamYPosition;
    double visionCamDeltaTime;

    public boolean getVisionCamAvailable(){
        return visionCamAvailable;
    }

    public boolean getVisionCamHasTarget(){
        return visionCamHasTarget;
    }

    public double getVisionCamYPosition(){
        return visionCamYPosition;
    }

    public double getVisionCamZPosition(){
        return visionCamZPosition;
    }
    //#endregion


    @Override
    public synchronized void onStart(double timestamp) {
        SmartDashboard.putString("RawVisionCamData_Raw", "0");
        blanks = 0;
        attemptVisionCamConnection();
        visionCamHasTarget = false;
        //for(int i=0; i<1000; i++){
            System.out.println("onStart called");
        //}
    }

    private int blanks;

    @Override
    public synchronized void onLoop(double timestamp) {
        if(!visionCamAvailable){
            attemptVisionCamConnection();
        }
        if(visionCamAvailable){
            String visionTargetPositions_Raw = visionCam.readString();
            System.out.println(visionTargetPositions_Raw);

            String[] visionTargetLines = visionTargetPositions_Raw.split("\n");
            ArrayList<TargetInfo> targetInfoArray = new ArrayList<>();
            for(int i = visionTargetLines.length-1; i >= 0; i--){
                boolean isValid = false;
                try {
                    JSONParser parser = new JSONParser();
                    JSONObject j = (JSONObject) parser.parse(visionTargetLines[i]);
                    visionCamDeltaTime = Double.parseDouble((String) j.get("DeltaTime"));
                    visionCamYPosition = Double.parseDouble((String) j.get("Y"));
                    visionCamZPosition = Double.parseDouble((String) j.get("Z"));
                    isValid = true;
                } catch(Exception e){
                    System.err.println(e.toString());
                }

                if(isValid){
                    TargetInfo targetInfo = new TargetInfo(visionCamYPosition, visionCamZPosition);
                    targetInfoArray.add(targetInfo);
                }
            }

            if(targetInfoArray.size() > 0){
                mRobotState.addVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray);
            }

            //#region Old Method
            /*
            String[] visionTargetPositions = visionTargetPositions_Raw.split(";");
            if(visionTargetPositions.length > 0){
                try{
                    String stringValue = visionTargetPositions[0].trim();
                    if(stringValue.length() > 0){
                        visionCamXPosition = Double.parseDouble(stringValue);
                        visionCamHasTarget = true;
                        blanks = 0;
                    }
                    else{
                        blanks++;
                    }
                }
                catch(Exception e){
                    System.out.println(e);
                    visionCamHasTarget = false;
                }
            }
            if(blanks > 5){
                visionCamHasTarget = false;
                blanks = 0;
            }
            if(visionCamHasTarget){
                SmartDashboard.putNumber("visionCamXPosition", visionCamXPosition);
            }
            else{
                SmartDashboard.putString("visionCamXPosition", "NO DATA");
            }
            */
            //#endregion
        }
        else{
            System.out.println("No vision cam");
        }
    }

    @Override
    public void onStop(double timestamp) {
        visionCamHasTarget = false;
    }

}
