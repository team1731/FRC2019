package org.usfirst.frc.team1731.robot.paths;

import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MirrorablePath implements PathContainer {

    private static int FIELD_WIDTH_INCHES = 27 * 12;

    private boolean isMirrored;

    public MirrorablePath(){
        isMirrored = "R".equals(SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode));
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