package org.usfirst.frc.team1731.lib.util.control;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * https://www.youtube.com/watch?v=_mKlRbapkXo
 */
@Deprecated
public class VisionDrive extends TimedRobot {

    private SerialPort visionCam = new SerialPort(115200, SerialPort.Port.kUSB1);

    @Override
    public void robotInit() {
        /*
        String[] visionTargetPosition = visionCam.readString().split(",");
        if(visionTargetPosition.length > 0){
            System.out.println("x: "+visionTargetPosition[0]+ ", y: "+visionTargetPosition[1]);
        } else {
            System.out.println("No data received from vision camera");
        }
        //x,y
        */
    }

}