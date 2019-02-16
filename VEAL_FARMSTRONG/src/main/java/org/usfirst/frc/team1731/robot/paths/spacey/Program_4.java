package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Program_4 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(260,200,0,0));
        sWaypoints.add(new Waypoint(250,240,25,60));
        sWaypoints.add(new Waypoint(120,240,25,60));
        sWaypoints.add(new Waypoint(90,290,25,60));
        sWaypoints.add(new Waypoint(20,300,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(260, 200), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":260,"y":200},"speed":0,"radius":0,"comment":""},{"position":{"x":250,"y":240},"speed":60,"radius":25,"comment":""},{"position":{"x":120,"y":240},"speed":60,"radius":25,"comment":""},{"position":{"x":90,"y":290},"speed":60,"radius":25,"comment":""},{"position":{"x":20,"y":300},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program 4
}