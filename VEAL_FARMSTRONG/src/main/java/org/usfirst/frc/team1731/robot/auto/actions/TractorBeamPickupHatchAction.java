package org.usfirst.frc.team1731.robot.auto.actions;

import java.util.Optional;

import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.ShooterAimingParameters;
import org.usfirst.frc.team1731.robot.subsystems.Drive;
import org.usfirst.frc.team1731.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

/**
 * Waits for the robot to pass by a provided path marker (i.e. a waypoint on the field). This action routinely compares
 * to the crossed path markers provided by the drivetrain (in Path Control mode) and returns if the parameter path
 * marker is inside the drivetrain's Path Markers Crossed list
 * 
 * @param A
 *            Path Marker to determine if crossed
 */
public class TractorBeamPickupHatchAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private String mMarker;
  


    /*public WaitForPathMarkerAction(String marker) {
        mMarker = marker;
    }
    */

    @Override
    public boolean isFinished() {
        return ( mDrive.isTBFinished());
       
    }

    @Override
    public void update() {
     //   if (mDrive.hasPassedMarker("StartTractorBeam")) {
           // System.out.println("Starting Tractor Beam");
       //    double now = Timer.getFPGATimestamp();
          // Optional<ShooterAimingParameters> aimParams = mRobotState.getAimingParameters();
         //  if (aimParams.isPresent() && Math.abs(now - aimParams.get().getLastSeenTimestamp()) < 0.5) {
               mDrive.setWantTractorBeam();
        //   }
        }
  

    @Override
    public void done() {
        if (mDrive.isTBFinished()) {
            mSuperstructure.openBeak();
        }

    }

    @Override
    public void start() {
        mSuperstructure.prepareToPickupHatch();
        mDrive.resetTractorBeam();
    }

}
