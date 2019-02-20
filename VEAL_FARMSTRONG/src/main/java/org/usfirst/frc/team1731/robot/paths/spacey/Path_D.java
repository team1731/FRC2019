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
 * @deprecated Path_D is split into {@link #Path_D_A()} and {@link #Path_D_B()}
 */
@Deprecated
public class Path_D implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(200,150,0,0)); //Turn 
        sWaypoints.add(new Waypoint(170,200,40,60));
        sWaypoints.add(new Waypoint(170,270,30,60));
        sWaypoints.add(new Waypoint(130,295,20,60));
        sWaypoints.add(new Waypoint(20,295,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(200, 150), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":200,"y":150},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":170,"y":200},"speed":60,"radius":40,"comment":""},{"position":{"x":170,"y":270},"speed":60,"radius":30,"comment":""},{"position":{"x":130,"y":295},"speed":60,"radius":20,"comment":""},{"position":{"x":20,"y":295},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program D
}