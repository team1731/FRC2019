package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Path_A_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(61,265,0,0)); //Turn 
        sWaypoints.add(new Waypoint(61,295,20,30));
        sWaypoints.add(new Waypoint(140,295,25,60));
        sWaypoints.add(new Waypoint(170,250,30,60));
        sWaypoints.add(new Waypoint(250,230,20,60));
        sWaypoints.add(new Waypoint(299,276,15,60));
        sWaypoints.add(new Waypoint(270,295,0,45));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(61, 265), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":61,"y":265},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":61,"y":295},"speed":30,"radius":20,"comment":""},{"position":{"x":140,"y":295},"speed":60,"radius":25,"comment":""},{"position":{"x":170,"y":250},"speed":60,"radius":30,"comment":""},{"position":{"x":250,"y":230},"speed":60,"radius":20,"comment":""},{"position":{"x":299,"y":276},"speed":60,"radius":15,"comment":""},{"position":{"x":270,"y":295},"speed":45,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_A_B
}