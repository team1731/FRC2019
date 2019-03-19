package org.usfirst.frc.team1731.robot.paths;
import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;



public class DriveForward extends MirrorablePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20, getY(250),  0,  0));
        sWaypoints.add(new Waypoint(35, getY(250), 15, 60));
        sWaypoints.add(new Waypoint(44, getY(220),  0, 30));
        sWaypoints.add(new Waypoint(44, getY(210),  0,  0));
        
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, getY(250)), Rotation2d.fromDegrees(getAngle(0))); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":250},"speed":0,"radius":0,"comment":""},{"position":{"x":80,"y":250},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: UntitledPath
}