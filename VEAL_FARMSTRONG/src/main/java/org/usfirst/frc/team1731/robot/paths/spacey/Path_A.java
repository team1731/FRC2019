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
 * 
 * @deprecated Path_A is split into {@link #Path_A_A()} and {@link #Path_A_B()}
 */
@Deprecated
public class Path_A implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(20,295,0,0)); //Turn 
        sWaypoints.add(new Waypoint(140,295,25,60));
        sWaypoints.add(new Waypoint(170,250,30,60));
        sWaypoints.add(new Waypoint(200,250,0,60));
        sWaypoints.add(new Waypoint(250,250,45,60));
        sWaypoints.add(new Waypoint(275,300,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 295), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":295},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":140,"y":295},"speed":60,"radius":25,"comment":""},{"position":{"x":170,"y":250},"speed":60,"radius":30,"comment":""},{"position":{"x":200,"y":250},"speed":60,"radius":0,"comment":""},{"position":{"x":250,"y":250},"speed":60,"radius":45,"comment":""},{"position":{"x":275,"y":300},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program A
}