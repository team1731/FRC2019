package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Program_3 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(280,230,0,0));
        sWaypoints.add(new Waypoint(160,240,25,60));
        sWaypoints.add(new Waypoint(120,300,25,60));
        sWaypoints.add(new Waypoint(20,300,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(280, 230), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":280,"y":230},"speed":0,"radius":0,"comment":""},{"position":{"x":160,"y":240},"speed":60,"radius":25,"comment":""},{"position":{"x":120,"y":300},"speed":60,"radius":25,"comment":""},{"position":{"x":20,"y":300},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program 3
}