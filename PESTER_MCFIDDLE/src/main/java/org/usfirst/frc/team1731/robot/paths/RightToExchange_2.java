package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class RightToExchange_2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(120,47,0,0));
        sWaypoints.add(new Waypoint(60,47,10,80));
        sWaypoints.add(new Waypoint(60,120,0,80));
        sWaypoints.add(new Waypoint(60,192,10,80));
        sWaypoints.add(new Waypoint(25,192,0,80));


        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(120, 47), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":50},"speed":0,"radius":0,"comment":""},{"position":{"x":210,"y":50},"speed":110,"radius":30,"comment":""},{"position":{"x":200,"y":60},"speed":30,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightToRightSwitch_A
}