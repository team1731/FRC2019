package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Program_6 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(265,30,0,0));
        sWaypoints.add(new Waypoint(285,50,25,60));
        sWaypoints.add(new Waypoint(265,90,20,60));
        sWaypoints.add(new Waypoint(180,120,25,60));
        sWaypoints.add(new Waypoint(150,35,25,60));
        sWaypoints.add(new Waypoint(22,35,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(265, 30), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":265,"y":30},"speed":0,"radius":0,"comment":""},{"position":{"x":285,"y":50},"speed":60,"radius":25,"comment":""},{"position":{"x":265,"y":90},"speed":60,"radius":20,"comment":""},{"position":{"x":180,"y":120},"speed":60,"radius":25,"comment":""},{"position":{"x":150,"y":35},"speed":60,"radius":25,"comment":""},{"position":{"x":22,"y":35},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Program 6
}