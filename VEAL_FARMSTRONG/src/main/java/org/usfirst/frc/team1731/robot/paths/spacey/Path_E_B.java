package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class Path_E_B implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(61,265,0,0)); //Turn 
        sWaypoints.add(new Waypoint(91,295,20,60));
        sWaypoints.add(new Waypoint(150,275,15,60));
        sWaypoints.add(new Waypoint(150,150,20,60));
        sWaypoints.add(new Waypoint(200,150,0,60));

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
	// WAYPOINT_DATA: [{"position":{"x":61,"y":265},"speed":0,"radius":0,"comment":"Turn "},{"position":{"x":91,"y":295},"speed":60,"radius":20,"comment":""},{"position":{"x":150,"y":275},"speed":60,"radius":15,"comment":""},{"position":{"x":150,"y":150},"speed":60,"radius":20,"comment":""},{"position":{"x":200,"y":150},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_E_B
}