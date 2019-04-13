package org.usfirst.frc.team1731.robot.auto.modes;

import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MirrorableMode extends AutoModeBase {

    private static int FIELD_WIDTH_INCHES = 27 * 12;

    private boolean isMirrored;

    public MirrorableMode(){
        String autoCode = SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode).toUpperCase().trim();
        System.out.println("MirrorableMode: autoCode=" + autoCode);
        isMirrored = "R".equals(autoCode);
        System.out.println("MirrorableMode: isMirrored=" + isMirrored);
    }

    protected int getY(int unMirroredYValue){
        if(isMirrored){
            return FIELD_WIDTH_INCHES - unMirroredYValue;
        }
        else{
            return unMirroredYValue;
        }
    }

    protected double getAngle(double unMirroredAngle){
        if(isMirrored){
            return -unMirroredAngle;
        }
        else{
            return unMirroredAngle;
        }
    }
}