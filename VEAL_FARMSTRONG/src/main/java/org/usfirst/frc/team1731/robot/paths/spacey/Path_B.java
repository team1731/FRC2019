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
 * @deprecated Path_B is split into {@link #Path_B_A()} and {@link #Path_B_B()}
 */
@Deprecated
public class Path_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,300,0,0));
        sWaypoints.add(new Waypoint(40,290,5,60));
        sWaypoints.add(new Waypoint(50,270,3,60));
        sWaypoints.add(new Waypoint(70,250,5,60));
        sWaypoints.add(new Waypoint(110,250,9,60));
        sWaypoints.add(new Waypoint(150,260,2,60));
        sWaypoints.add(new Waypoint(210,290,0,60));

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
	// WAYPOINT_DATA: [{"position":{"x":20,"y":300},"speed":0,"radius":0,"comment":""},{"position":{"x":40,"y":290},"speed":60,"radius":5,"comment":""},{"position":{"x":50,"y":270},"speed":60,"radius":3,"comment":""},{"position":{"x":70,"y":250},"speed":60,"radius":5,"comment":""},{"position":{"x":110,"y":250},"speed":60,"radius":9,"comment":""},{"position":{"x":150,"y":260},"speed":60,"radius":2,"comment":""},{"position":{"x":210,"y":290},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program B
}