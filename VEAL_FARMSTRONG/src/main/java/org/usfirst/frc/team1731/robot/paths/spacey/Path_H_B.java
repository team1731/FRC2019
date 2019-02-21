package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Path_H_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(61,65,0,0)); //Turn 
        sWaypoints.add(new Waypoint(120,35,20,60));
        sWaypoints.add(new Waypoint(180,75,15,60));
        sWaypoints.add(new Waypoint(263,75,20,60));
        sWaypoints.add(new Waypoint(263,120,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(61, 65), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":61,"y":65},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":120,"y":35},"speed":60,"radius":20,"comment":""},{"position":{"x":180,"y":75},"speed":60,"radius":15,"comment":""},{"position":{"x":263,"y":75},"speed":60,"radius":20,"comment":""},{"position":{"x":263,"y":120},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_H_B
}