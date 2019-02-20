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
 * @deprecated Path_C is split into {@link #Path_C_A()} and {@link #Path_C_B()}
 */
@Deprecated
public class Path_C implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,300,0,0));
        sWaypoints.add(new Waypoint(60,300,25,60));
        sWaypoints.add(new Waypoint(90,270,25,60));
        sWaypoints.add(new Waypoint(190,270,25,60));
        sWaypoints.add(new Waypoint(260,230,25,60));
        sWaypoints.add(new Waypoint(260,200,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 300), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":300},"speed":0,"radius":0,"comment":""},{"position":{"x":60,"y":300},"speed":60,"radius":25,"comment":""},{"position":{"x":90,"y":270},"speed":60,"radius":25,"comment":""},{"position":{"x":190,"y":270},"speed":60,"radius":25,"comment":""},{"position":{"x":260,"y":230},"speed":60,"radius":25,"comment":""},{"position":{"x":260,"y":200},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program C
}