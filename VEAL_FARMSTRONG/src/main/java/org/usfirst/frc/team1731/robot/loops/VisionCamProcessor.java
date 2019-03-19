package org.usfirst.frc.team1731.robot.loops;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
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

    private SerialPort visionCam;

    private void attemptVisionCamConnection(){
        SmartDashboard.putBoolean("visionCamConnected", false);
        try {
            if(visionCam == null){
                visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
            }
            if(visionCam != null){
                //visionCam.writeString("streamoff\n");
                //visionCam.writeString("usbsd\n");
                visionCamAvailable = true;
                SmartDashboard.putBoolean("visionCamConnected", true);
            }
        } catch(Exception e){
            System.out.println(e.toString());
        }
    }

    public void setVisionCam(SerialPort visionCam){
        this.visionCam = visionCam;
        visionCamAvailable = true;
    }

    //#region Vision Camera Variables
    boolean visionCamAvailable;
    boolean visionCamHasTarget;
    int visionCamXPosition;

    public boolean getVisionCamAvailable(){
        return visionCamAvailable;
    }

    public boolean getVisionCamHasTarget(){
        return visionCamHasTarget;
    }

    public int getVisionCamXPosition(){
        return visionCamXPosition;
    }
    //#endregion


    @Override
    public synchronized void onStart(double timestamp) {
        SmartDashboard.putString("RawVisionCamData_Raw", "0");
        blanks = 0;
        attemptVisionCamConnection();
        visionCamHasTarget = false;
        for(int i=0; i<1000; i++){
            System.out.println("onStart called");
        }
    }

    private int blanks;

    @Override
    public synchronized void onLoop(double timestamp) {
        if(!visionCamAvailable){
            attemptVisionCamConnection();
        }
        if(visionCamAvailable){
            String visionTargetPositions_Raw = visionCam.readString().trim();
            System.out.println(visionTargetPositions_Raw);
            String[] visionTargetPositions = visionTargetPositions_Raw.split(";");
            if(visionTargetPositions.length > 0){
                try{
                    String stringValue = visionTargetPositions[0].trim();
                    if(stringValue.length() > 0){
                        visionCamXPosition = Integer.parseInt(stringValue);
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
