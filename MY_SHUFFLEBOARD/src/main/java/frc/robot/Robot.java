package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
    private SerialPort visionCam;

    @Override
    public void robotInit() {
      try {
        CameraServer.getInstance().startAutomaticCapture(0);
    }
    catch (Throwable t) {
        System.out.println("Exception while connecting driver camera: " + t.toString());
    }
    try {
        String line;
        visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);

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
        
        //setpar serout USB
        visionCam.writeString("setpar serout USB\n"); visionCam.flush();        
        System.out.println("setpar serout USB"); System.out.flush();
        while((line = visionCam.readString()).length() > 0){
            System.out.print(line.trim()); System.out.flush();
        }

        //streamon
        visionCam.writeString("streamon\n"); visionCam.flush();        
        System.out.println("streamon"); System.out.flush();
        while((line = visionCam.readString()).length() > 0){
            System.out.print(line.trim()); System.out.flush();
        }

        //setpar serout USB
        //visionCam.writeString("setpar serout USB\n"); visionCam.flush();        
        //System.out.println("setpar serout USB"); System.out.flush();
        //while((line = visionCam.readString()).length() > 0){
        //    System.out.print(line.trim()); System.out.flush();
        //}

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

        //Calibration: 85,65,255,200,255,200,0,3,5,100,100
        //uh = int(CalFile[0])
        //lh = int(CalFile[1])
        //us = int(CalFile[2])
        //ls = int(CalFile[3])
        //uv = int(CalFile[4])
        //lv = int(CalFile[5])
        //er = int(CalFile[6])
        //dl = int(CalFile[7])
        //ap = int(CalFile[8])
        //ar = int(CalFile[9])
        //sl = float(CalFile[10])
        int[] calibration = {85,65,255,200,255,200,0,3,5,100,100};
        String[] parameters = {"uh", "lh", "us", "ls", "uv", "lv", "er", "dl", "ap", "ar", "sl"};
        for(int i=0; i<calibration.length; i++){
            visionCam.writeString("setpar "+ parameters[i] + " " + calibration[i] + "\n"); visionCam.flush();        
            System.out.println("setpar "+ parameters[i] + " " + calibration[i]); System.out.flush();
            while((line = visionCam.readString()).length() > 0){
                System.out.print(line.trim()); System.out.flush();
            }    
        }
    }
    catch (Throwable t) {
        System.out.println("Exception while connecting vision camera: " + t.toString());
    }
} 

    @Override
    public void teleopPeriodic(){
        if(visionCam != null){
            String raw = visionCam.readString().trim();
            if(raw.length() > 0){
                System.out.println("received: '" + raw + "'");
            }
        }
    }
}
