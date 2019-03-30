package org.usfirst.frc.team1731.robot.vision;


import java.util.List;


/**
 * VisionUpdate contains the various attributes outputted by the vision system, namely a list of targets and the
 * timestamp at which it was captured.
 */
public class JevoisVisionUpdate {

    protected List<TargetInfo> mtargets;
    protected double mCapturedAtTimestamp = 0;

    public JevoisVisionUpdate(double capturedAtTimestamp, List<TargetInfo> targets) {
        mtargets = targets;
        mCapturedAtTimestamp = capturedAtTimestamp;

    }
    public List<TargetInfo> getTargets() {
        return mtargets;
    }



    public double getCapturedAtTimestamp() {
        return mCapturedAtTimestamp;
    }

}
