package org.usfirst.frc.team1731.robot.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.SerialPort;
import org.json.simple.parser.JSONParser;

import org.usfirst.frc.team1731.lib.util.CrashTrackingRunnable;
import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.loops.JevoisVisionProcessor;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This controls all vision actions, including vision updates, capture, and interfacing with the Android phone with
 * Android Debug Bridge. It also stores all VisionUpdates (from the Android phone) and contains methods to add to/prune
 * the VisionUpdate list. Much like the subsystems, outside methods get the VisionServer instance (there is only one
 * VisionServer) instead of creating new VisionServer instances.
 * 
 * @see VisionUpdate.java
 */

public class JevoisVisionServer {

    private static JevoisVisionServer s_instance = null;
    private boolean m_running = true;
    double lastMessageReceivedTime = 0;
    private boolean m_use_java_time = false;
    private SerialPort visionCam;
    boolean visionCamAvailable;
    boolean visionCamHasTarget;
    double visionCamZPosition;
    double visionCamYPosition;
    double visionCamDeltaTime;

 //   private ArrayList<ServerThread> serverThreads = new ArrayList<>();
 //   private volatile boolean mWantsAppRestart = false;

    public static JevoisVisionServer getInstance() {
        if (s_instance == null) {
            s_instance = new JevoisVisionServer();

        }
        return null;// s_instance;
    }


    private class VisionServerThread extends CrashTrackingRunnable {

        private JevoisVisionProcessor mJevoisVisionProcessor = JevoisVisionProcessor.getInstance();

        private VisionServerThread(){
            try {
                AttemptJevoisConnection();
            } catch (Exception e){
                try {
                    Thread.sleep(1000);
                } catch(InterruptedException re){
                    re.printStackTrace();
                }
                AttemptJevoisConnection();
            }
        }

        private boolean SafeToParse(String message){
            if(message.length() > 1){
                return message.charAt(0) == '{' && message.charAt(message.length()-3) == '}' && message.contains("{\"DeltaTime\"") && message.contains("\"Y\"") && message.contains("\"Z\"");
            } else {
                return false;
            }
        }

        int dashboardCounter = 0;
        int sentTimes = 0;
        public void handleMessage(String message, double timestamp) {
            dashboardCounter++;
            String visionTargetPositions_Raw = message; //message should be the raw visionCam.readString() called from runCrashTracked()
            //System.out.println(visionTargetPositions_Raw);
            if(!SafeToParse(visionTargetPositions_Raw)){
                return;
            }
            
            if(dashboardCounter >= 10){
              //  SmartDashboard.putString("JevoisVisionServerTargets", visionTargetPositions_Raw);
            }

            String[] visionTargetLines = visionTargetPositions_Raw.split("\n");
            ArrayList<TargetInfo> targetInfoArray = new ArrayList<>();
            for(int i = visionTargetLines.length-1; i >= 0; i--){
                boolean isValid = false;
                try {
                    String thisTargetLine = visionTargetLines[i];
                    // if(SafeToParse(thisTargetLine, true)){
                    JSONParser parser = new JSONParser();
                    JSONObject j = (JSONObject) parser.parse(thisTargetLine);
                    visionCamDeltaTime = Double.parseDouble((String) j.get("DeltaTime"));
                    visionCamYPosition = Double.parseDouble((String) j.get("Y"));
                    visionCamZPosition = Double.parseDouble((String) j.get("Z"));
                    if (visionCamYPosition == 0.0 && visionCamZPosition == 0.0) {
                        isValid = false;
                    } else {
                        isValid = true;
                    }
                    //}
                } catch(Exception e) {
                    System.err.println("Parse error: "+e.toString());
                }

                if(isValid){
                    TargetInfo targetInfo = new TargetInfo(visionCamYPosition, visionCamZPosition);
                    targetInfoArray.add(targetInfo);
                }
            }

            //System.out.println("TargetInfoArray.size(): "+targetInfoArray.size());

            if(targetInfoArray.size() > 0){ 
                sentTimes++;
                if(dashboardCounter >= 10){
                 //   SmartDashboard.putString("JevoisVisionServerUpdate", "Sent: "+sentTimes);
                }
                mJevoisVisionProcessor.gotUpdate(new JevoisVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray));

             //   mRobotState.addVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray);
            }

            if(dashboardCounter >= 10){
                dashboardCounter = 0;
            }

        }

        int connectionAttempt = 0; //Debug purposes

        private void AttemptJevoisConnection(){
            try {
                Thread.sleep(1000);
                connectionAttempt++;
               // SmartDashboard.putString("JevoisVisionServerOutput", "Attempting Jevois connection... ("+connectionAttempt+")");
                
                visionCam = new SerialPort(115200, SerialPort.Port.kMXP);
                visionCam.setTimeout(5);

                if(visionCam != null){
                 //   SmartDashboard.putString("JevoisVisionServerOutput", "(NO RUN) Connected successfully on attempt "+connectionAttempt);
                    visionCamAvailable = true;
                    
                    //run();
                } else {
                    visionCamAvailable = false;
                    AttemptJevoisConnection();
                }
                

            } catch(Exception e){
                visionCamAvailable = false;
                System.err.println("is this it? "+e.toString());
                e.printStackTrace();
                AttemptJevoisConnection();
            }
        }


        String lastDashboardMessage = "";

        @Override
        public void runCrashTracked() {
            while (true) {
                String dashboardMessage = "";
                visionCamAvailable = visionCam != null;
                try {
                    if(visionCamAvailable){
                        dashboardMessage = "visionCamAvailable == true. Handling message";
                        String visionTargetPositions_Raw = visionCam.readString();
                        handleMessage(visionTargetPositions_Raw, getTimestamp());
                    } else {
                        dashboardMessage = "visionCamAvailable == false. Lost connection";
                        AttemptJevoisConnection();
                        //return;
                        break;
                    }
                    if(lastDashboardMessage != dashboardMessage){
                      //  SmartDashboard.putString("JevoisVisionServerOutput", dashboardMessage);
                    }
                    lastDashboardMessage = dashboardMessage;
                } catch (Exception e){
                    try{
                        Thread.sleep(50);
                    } catch (InterruptedException ie){}
                    AttemptJevoisConnection();
                }
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

        


    /**
     * Instantializes the VisionServer and connects to ADB via the specified port.
     * 
     */
    private JevoisVisionServer() {
        new Thread(new VisionServerThread()).start();
    }





    private double getTimestamp() {
        if (m_use_java_time) {
            return System.currentTimeMillis();
        } else {
            return Timer.getFPGATimestamp();
        }
    }
}
