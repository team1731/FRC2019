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
 * Path_G is split into {@link #Path_G_A()} and {@link #Path_G_B()}
 */
public class Path_G implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(25,35,0,0));
        sWaypoints.add(new Waypoint(90,35,25,60));
        sWaypoints.add(new Waypoint(150,153,25,60));
        sWaypoints.add(new Waypoint(205,153,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(25, 35), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":25,"y":35},"speed":0,"radius":0,"comment":""},{"position":{"x":90,"y":35},"speed":60,"radius":25,"comment":""},{"position":{"x":150,"y":153},"speed":60,"radius":25,"comment":""},{"position":{"x":205,"y":153},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program G
}