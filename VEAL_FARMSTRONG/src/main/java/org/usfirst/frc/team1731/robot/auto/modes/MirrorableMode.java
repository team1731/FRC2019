package org.usfirst.frc.team1731.robot.auto.modes;

import org.usfirst.frc.team1731.robot.Constants;
import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.paths.PathContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MirrorableMode extends AutoModeBase {

    private static int FIELD_WIDTH_INCHES = 27 * 12;

    // check the value every time instead of using a constructor because
    // auto modes only get instantiated once when the robot code
    // first starts and the drive team might change the value in the
    // shuffleboard after that!
    private boolean isMirrored(){
        String autoCode = SmartDashboard.getString("AutoCode", Constants.kDefaultAutoMode).toUpperCase().trim();
        boolean isMirrored = "R".equals(autoCode);
        return isMirrored;
    }

    protected int getY(int unMirroredYValue){
        if(isMirrored()){
            return FIELD_WIDTH_INCHES - unMirroredYValue;
        }
        else{
            return unMirroredYValue;
        }
    }

    protected double getAngle(double unMirroredAngle){
        if(isMirrored()){
            return -unMirroredAngle;
        }
        else{
            return unMirroredAngle;
        }
    }
}