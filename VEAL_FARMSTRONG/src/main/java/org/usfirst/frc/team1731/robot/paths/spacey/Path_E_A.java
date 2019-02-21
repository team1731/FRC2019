package org.usfirst.frc.team1731.robot.paths.spacey;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

//EA GAMES!! CHALLENGE EVVVVERYTTHINGNGGGGGG
/**
 * Should be followed by {@link #Path_E_B()}
 */
public class Path_E_A implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(31,295,0,0));
        sWaypoints.add(new Waypoint(61,295,15,60));
        sWaypoints.add(new Waypoint(61,265,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(31, 295), Rotation2d.fromDegrees(0.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":31,"y":295},"speed":0,"radius":0,"comment":""},{"position":{"x":61,"y":295},"speed":60,"radius":15,"comment":""},{"position":{"x":61,"y":265},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
    // FILE_NAME: Path_E_A
}