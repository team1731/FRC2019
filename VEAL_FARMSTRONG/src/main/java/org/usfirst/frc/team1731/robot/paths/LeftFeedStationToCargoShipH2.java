package org.usfirst.frc.team1731.robot.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1731.lib.util.control.Path;
import org.usfirst.frc.team1731.lib.util.math.RigidTransform2d;
import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Translation2d;
import org.usfirst.frc.team1731.robot.paths.PathBuilder;
import org.usfirst.frc.team1731.robot.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team1731.robot.paths.PathContainer;

public class LeftFeedStationToCargoShipH2 extends MirrorablePath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(20, getY(295), 0,   0));
        sWaypoints.add(new Waypoint(290, getY(245), 0, 120));
   //     sWaypoints.add(new Waypoint(240, getY(265), 0,  60));
//sWaypoints.add(new Waypoint(255, getY(260), 5,  60));
   //     sWaypoints.add(new Waypoint(260, getY(250), 0,  60));
 //       sWaypoints.add(new Waypoint(260, getY(190), 0,  30));      
        
/* before 
        sWaypoints.add(new Waypoint(0,200,0,0));
        sWaypoints.add(new Waypoint(48,200,0,60));
        sWaypoints.add(new Waypoint(104,225,0,110));
        sWaypoints.add(new Waypoint(144,246,0,80));
        sWaypoints.add(new Waypoint(193,216,0,60));
        sWaypoints.add(new Waypoint(200,189,0,60));
worked twice in a row - but slow and coordinates goofy
        sWaypoints.add(new Waypoint(0,200,0,0));
        sWaypoints.add(new Waypoint(48,200,0,30));
        sWaypoints.add(new Waypoint(104,225,0,60));
        sWaypoints.add(new Waypoint(144,246,0,60));
        sWaypoints.add(new Waypoint(195,216,0,60));
        sWaypoints.add(new Waypoint(203,189,0,60));
*/
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, getY(295)), Rotation2d.fromDegrees(getAngle(180.0))); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":19,"y":295},"speed":60,"radius":0,"comment":""},{"position":{"x":55,"y":295},"speed":0,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Path_1
}