package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class LeftRocketRearToFeedStationPath2 extends MirrorablePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        //sWaypoints.add(new Waypoint(314, getY(272),  0, 70));
        sWaypoints.add(new Waypoint(295, getY(290), 0, 0));
        sWaypoints.add(new Waypoint(200, getY(250), 0, 90));
        sWaypoints.add(new Waypoint(130, getY(295), 0, 90));
        sWaypoints.add(new Waypoint(100, getY(295), 0, 90,"StartTractorBeam"));
        sWaypoints.add(new Waypoint(70, getY(295), 0, 70));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(295, getY(290)), Rotation2d.fromDegrees(getAngle(0))); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":55,"y":295},"speed":60,"radius":0,"comment":""},{"position":{"x":19,"y":295},"speed":0,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_2
}