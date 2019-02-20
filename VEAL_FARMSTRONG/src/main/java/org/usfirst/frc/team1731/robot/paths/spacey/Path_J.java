package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

/**
 * Path_J is split into {@link #Path_J_A()} and {@link #Path_J_B()}
 */
public class Path_J implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(275,32,0,0)); //Turn 
        sWaypoints.add(new Waypoint(230,100,45,60));
        sWaypoints.add(new Waypoint(180,100,10,60));
        sWaypoints.add(new Waypoint(140,70,20,60));
        sWaypoints.add(new Waypoint(25,32,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(275, 32), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":275,"y":32},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":230,"y":100},"speed":60,"radius":45,"comment":""},{"position":{"x":180,"y":100},"speed":60,"radius":10,"comment":""},{"position":{"x":140,"y":70},"speed":60,"radius":20,"comment":""},{"position":{"x":25,"y":32},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program J
}