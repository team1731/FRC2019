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
 * Path_E is split into {@link #Path_E_A()} and {@link #Path_E_B()}
 */
public class Path_E implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(25,295,0,0));
        sWaypoints.add(new Waypoint(80,295,25,60));
        sWaypoints.add(new Waypoint(120,180,25,60));
        sWaypoints.add(new Waypoint(160,150,25,60));
        sWaypoints.add(new Waypoint(205,150,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(25, 295), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":25,"y":295},"speed":0,"radius":0,"comment":""},{"position":{"x":80,"y":295},"speed":60,"radius":25,"comment":""},{"position":{"x":120,"y":180},"speed":60,"radius":25,"comment":""},{"position":{"x":160,"y":150},"speed":60,"radius":25,"comment":""},{"position":{"x":205,"y":150},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program E
}