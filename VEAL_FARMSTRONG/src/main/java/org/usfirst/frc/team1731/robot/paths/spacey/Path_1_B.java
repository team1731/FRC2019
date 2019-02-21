package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Path_1_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(292,310,0,60));
        sWaypoints.add(new Waypoint(292,265,25,60));
        sWaypoints.add(new Waypoint(150,265,15,60));
        sWaypoints.add(new Waypoint(150,295,15,60));
        sWaypoints.add(new Waypoint(19,295,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(292, 310), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":292,"y":310},"speed":60,"radius":0,"comment":""},{"position":{"x":292,"y":265},"speed":60,"radius":25,"comment":""},{"position":{"x":150,"y":265},"speed":60,"radius":15,"comment":""},{"position":{"x":150,"y":295},"speed":60,"radius":15,"comment":""},{"position":{"x":19,"y":295},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_1_B
}