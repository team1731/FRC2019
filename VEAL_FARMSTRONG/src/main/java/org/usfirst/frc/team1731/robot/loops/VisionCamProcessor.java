package org.usfirst.frc.team1731.robot.loops;

import org.usfirst.frc.team1731.robot.ControlBoardInterface;
import org.usfirst.frc.team1731.robot.GamepadControlBoard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class VisionCamProcessor implements Loop {
    static VisionCamProcessor instance_ = new VisionCamProcessor();

    public static VisionCamProcessor getInstance() {
        return instance_;
    }

    VisionCamProcessor() {
    }

    //#region Vision Camera Variables
    boolean visionCamAvailable = false;
    double visionCamXPosition;

    public boolean GetVisionCamAvailable(){
        return visionCamAvailable;
    }

    public double GetVisionCamXPosition(){
        return visionCamXPosition;
    }
    //#endregion


    @Override
    public synchronized void onStart(double timestamp) {
        SmartDashboard.putString("RawVisionCamData_Raw", "0");
        SmartDashboard.putString("SentVisionCamData", "0");
        AttemptVisionCamConnection();
    }

    private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();
    private SerialPort visionCam;

    private void AttemptVisionCamConnection(){
        try {
            visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
        } catch(Exception e){
            visionCam = null;
            visionCamAvailable = false;
            System.out.println(e.toString());
        }
    }

    @Override
    public synchronized void onLoop(double timestamp) {

        boolean tracktorDrive = mControlBoard.getTractorDrive();
        if(visionCam == null){
            visionCamAvailable = false;
            AttemptVisionCamConnection();
        } else {
            String visionTargetPositions_Raw = visionCam.readString();
            String[] visionTargetPositions = visionTargetPositions_Raw.split(";");
            visionCamAvailable = visionTargetPositions[0].length() > 0 && visionTargetPositions[0] != null;
            int wantedIndex = visionTargetPositions.length-1;
            while(visionTargetPositions[wantedIndex].length() != 3 && visionTargetPositions[wantedIndex].length() != 5){
                if(wantedIndex <= 0){
                    break;
                }
                wantedIndex--;
            }
            SmartDashboard.putString("RawVisionCamData_Raw", visionTargetPositions_Raw);
            SmartDashboard.putString("SentVisionCamData", visionTargetPositions[wantedIndex]);
            try {
                visionCamXPosition = Double.valueOf(visionTargetPositions[wantedIndex]);
            } catch(Exception e){
                visionCamAvailable = false;
                visionCamXPosition = -1;
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
