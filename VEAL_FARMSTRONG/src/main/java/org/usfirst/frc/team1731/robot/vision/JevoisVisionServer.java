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
import org.usfirst.frc.team1731.robot.vision.messages.HeartbeatMessage;
import org.usfirst.frc.team1731.robot.vision.messages.OffWireMessage;
import org.usfirst.frc.team1731.robot.vision.messages.VisionMessage;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.Timer;

/**
 * This controls all vision actions, including vision updates, capture, and interfacing with the Android phone with
 * Android Debug Bridge. It also stores all VisionUpdates (from the Android phone) and contains methods to add to/prune
 * the VisionUpdate list. Much like the subsystems, outside methods get the VisionServer instance (there is only one
 * VisionServer) instead of creating new VisionServer instances.
 * 
 * @see VisionUpdate.java
 */

public class JevoisVisionServer extends CrashTrackingRunnable {

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


        private JevoisVisionProcessor mJevoisVisionProcessor = JevoisVisionProcessor.getInstance();

        public void handleMessage(String message, double timestamp) {
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
                mJevoisVisionProcessor.gotUpdate(new JevoisVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray));

             //   mRobotState.addVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray);
            }


        }

     

        @Override
        public void runCrashTracked() {
            while (true) {
            if(visionCamAvailable){
                String visionTargetPositions_Raw = visionCam.readString();
                handleMessage(visionTargetPositions_Raw,getTimestamp());
            }
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        }


    /**
     * Instantializes the VisionServer and connects to ADB via the specified port.
     * 
     * @param Port
     */
    private JevoisVisionServer() {

        try {
            visionCam = new SerialPort(115200, SerialPort.Port.kMXP);
            
            if(visionCam != null){
                //visionCam.writeString("streamoff\n");
                //visionCam.writeString("usbsd\n");
                visionCamAvailable = true;
                //SmartDashboard.putBoolean("visionCamConnected", true);
            }
        } catch(Exception e){
            System.out.println(e.toString());
        }

   
    }





    private double getTimestamp() {
        if (m_use_java_time) {
            return System.currentTimeMillis();
        } else {
            return Timer.getFPGATimestamp();
        }
    }
}
