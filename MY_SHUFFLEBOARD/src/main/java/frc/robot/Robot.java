package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
    private SerialPort visionCam;
    private int emptyCount;

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

            //setcam autoexp 1
            visionCam.writeString("setcam autoexp 1\n"); visionCam.flush();        
            System.out.println("setcam autoexp 1"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            //setcam absexp 75
            visionCam.writeString("setcam absexp 75\n"); visionCam.flush();        
            System.out.println("setcam absexp 75"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            //setpar serout USB
            visionCam.writeString("setpar serout USB\n"); visionCam.flush();        
            System.out.println("setpar serout USB"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            //setpar serlog None
            visionCam.writeString("setpar serlog None\n"); visionCam.flush();        
            System.out.println("setpar serlog None"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }

            //setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream
            visionCam.writeString("setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream\n"); visionCam.flush();
            System.out.println("setmapping2 YUYV 320 240 30.0 JeVois EagleTrkNoStream"); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }
            
            String[] calNames = {"uh","lh","us","ls","uv","lv","er","dl","ap","ar","sl"};
            int[] calValues = {85,65,255,200,255,200,0,3,5,100,100};
            for(int i=0; i<calNames.length; i++){
                visionCam.writeString("setpar " + calNames[i] + " " + calValues[i] + "\n"); visionCam.flush();        
                System.out.println("setpar " + calNames[i] + " " + calValues[i]); System.out.flush();
                while((line = visionCam.readString()).length() > 0){
                    System.out.print(line.trim()); System.out.flush();
                }    
            }
            //streamon
            visionCam.writeString("streamon\n"); visionCam.flush();        
            System.out.println("streamon"); System.out.flush();
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
    }

    @Override
    public void teleopPeriodic(){
        if(visionCam != null){
            String raw = visionCam.readString().trim();
            if(raw.length() > 0){
                //System.out.println("received: '" + raw + "'");
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
