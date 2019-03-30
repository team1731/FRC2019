package org.usfirst.frc.team1731.robot.loops;

import org.usfirst.frc.team1731.robot.GoalTracker;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.vision.JevoisVisionUpdate;
import org.usfirst.frc.team1731.robot.vision.VisionUpdate;
import org.usfirst.frc.team1731.robot.vision.VisionUpdateReceiver;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in RobotState. This helps keep track of goals
 * detected by the vision system. The code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class JevoisVisionProcessor implements Loop  {
    static JevoisVisionProcessor instance_ = new JevoisVisionProcessor();
    JevoisVisionUpdate update_ = null;
    RobotState robot_state_ = RobotState.getInstance();

    public static JevoisVisionProcessor getInstance() {
        return instance_;
    }

    JevoisVisionProcessor() {
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {

        JevoisVisionUpdate update;
        synchronized (this) {
            if (update_ == null) {
                return;
            }
            update = update_;
            update_ = null;
        }
        robot_state_.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

   
    public synchronized void gotUpdate(JevoisVisionUpdate update) {
        update_ = update;
    }

}
