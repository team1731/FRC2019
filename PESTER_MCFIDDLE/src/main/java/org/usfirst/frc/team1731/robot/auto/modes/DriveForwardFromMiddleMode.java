package org.usfirst.frc.team1731.robot.auto.modes;

import org.usfirst.frc.team1731.robot.auto.AutoModeBase;
import org.usfirst.frc.team1731.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team1731.robot.auto.actions.DrivePathAction;
import org.usfirst.frc.team1731.robot.auto.actions.ResetPoseFromPathAction;
import org.usfirst.frc.team1731.robot.paths.MiddleToRightSwitch;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

/**
 * When teammates are super trustworthy, autonomous will run this, simply driving past the line
 */
public class DriveForwardFromMiddleMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Ahh... just sit back and relax, our alliance's got this");
        PathContainer Path = new MiddleToRightSwitch();
    	runAction(new ResetPoseFromPathAction(Path));
    	runAction(new DrivePathAction(Path));
    }
}