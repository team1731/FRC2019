package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
    private SerialPort visionCam;
    private int emptyCount;
    private double teleopTimestampSave;
    private String saveSendToVisionCam = "info";

    private void setupDriverCamera(){
        try {
            SmartDashboard.putBoolean("DRIVER_CAM", false);
            CameraServer.getInstance().startAutomaticCapture(0);
            SmartDashboard.putBoolean("DRIVER_CAM", true);
        }
        catch (Throwable t) {
            System.out.println("Exception while connecting driver camera: " + t.toString());
        }
    }

    private void setupVisionCamera(){
        try {
            String line;
            SmartDashboard.putBoolean("VISION_CAM", false);
            visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);
            SmartDashboard.putBoolean("VISION_CAM", true);

            // //streamoff
            // visionCam.writeString("streamoff\n"); visionCam.flush();        
            // System.out.println("streamoff"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            // //restart
            // visionCam.writeString("restart\n"); visionCam.flush();        
            // System.out.println("restart"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            // Thread.sleep(1000);

            //info
            // visionCam.writeString("info\n"); visionCam.flush();        
            // System.out.println("info"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            //setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream
            visionCam.writeString("setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream\n"); visionCam.flush();
            System.out.println("setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }
            
            //setcam autoexp 1
            visionCam.writeString("setcam autoexp 1\n"); visionCam.flush();        
            System.out.println("setcam autoexp 1"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            //setcam absexp 75
            visionCam.writeString("setcam absexp 900\n"); visionCam.flush();        
            System.out.println("setcam absexp 900"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            // //setpar serout USB
            // visionCam.writeString("setpar serout USB\n"); visionCam.flush();        
            // System.out.println("setpar serout USB"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            // //setpar serlog None
            // visionCam.writeString("setpar serlog None\n"); visionCam.flush();        
            // System.out.println("setpar serlog None"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }
            
            // //setcam presetwb 1 (auto)
            // visionCam.writeString("setcam presetwb 1\n"); visionCam.flush();        
            // System.out.println("setcam presetwb 1"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            // //setcam autoexp 0
            // visionCam.writeString("setcam autoexp 0\n"); visionCam.flush();        
            // System.out.println("setcam autoexp 0"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            // //setcam absexp
            // visionCam.writeString("setcam absexp 900\n"); visionCam.flush();        
            // System.out.println("setcam absexp 900"); System.out.flush();
            // while((line = visionCam.readString()).length() > 0){
            //     System.out.print(line.trim()); System.out.flush();
            // }

            //getcam absexp
            visionCam.writeString("getcam absexp\n"); visionCam.flush();        
            System.out.println("getcam absexp"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

        }
        catch (Throwable t) {
            System.out.println("Exception while connecting/configuring vision camera: " + t.toString());
        }
        SmartDashboard.putBoolean("TARGET_LOCK", false);
    }

    @Override
    public void robotInit() {
        setupDriverCamera();
        setupVisionCamera();
        //CameraServer.getInstance().startAutomaticCapture(1); //for debug only! too much cpu and bw!
        SmartDashboard.putString("sendToVisionCam", "info");
    }

    @Override
    public void teleopPeriodic(){
        double teleopTimestamp = Timer.getFPGATimestamp();
        if((teleopTimestamp - teleopTimestampSave) >= 3){
            //adjustCameraValues();
            String sendToVisionCam = SmartDashboard.getString("sendToVisionCam", "info");
            if(!saveSendToVisionCam.equals(sendToVisionCam)){
                visionCam.writeString(sendToVisionCam + "\n"); visionCam.flush();
                System.out.println(sendToVisionCam); System.out.flush();
                String line;
                while((line = visionCam.readString()).length() > 0){
                    System.out.print(line.trim()); System.out.flush();
                }
                saveSendToVisionCam = sendToVisionCam;    
            }
            teleopTimestampSave = teleopTimestamp;
        }

        if(visionCam != null){
            String raw = visionCam.readString().trim();
            if(raw.length() > 0){
                System.out.println("received: '" + raw + "'");
                SmartDashboard.putString("TARGET_X", raw);
                emptyCount = 0;
                SmartDashboard.putBoolean("TARGET_LOCK", true);
            }
            else{
                emptyCount++;
                if(emptyCount > 5){
                    SmartDashboard.putBoolean("TARGET_LOCK", false);
                    SmartDashboard.putString("TARGET_X", "???");
                }
            }
        }
    }
}
