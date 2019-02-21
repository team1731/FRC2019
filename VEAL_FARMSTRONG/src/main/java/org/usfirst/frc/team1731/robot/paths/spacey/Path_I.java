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
 * Path_I is split into {@link #Path_I_A()} and {@link #Path_I_B()}
 */
public class Path_I implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(25,35,0,0));
        sWaypoints.add(new Waypoint(50,35,20,60));
        sWaypoints.add(new Waypoint(90,55,25,60));
        sWaypoints.add(new Waypoint(160,55,25,60));
        sWaypoints.add(new Waypoint(200,35,0,60));

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
	// WAYPOINT_DATA: [{"position":{"x":25,"y":35},"speed":0,"radius":0,"comment":""},{"position":{"x":50,"y":35},"speed":60,"radius":20,"comment":""},{"position":{"x":90,"y":55},"speed":60,"radius":25,"comment":""},{"position":{"x":160,"y":55},"speed":60,"radius":25,"comment":""},{"position":{"x":200,"y":35},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program I
}