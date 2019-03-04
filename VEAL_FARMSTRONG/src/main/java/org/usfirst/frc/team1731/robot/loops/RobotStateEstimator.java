package org.usfirst.frc.team1731.robot.loops;

import org.usfirst.frc.team1731.lib.util.math.Rotation2d;
import org.usfirst.frc.team1731.lib.util.math.Twist2d;
import org.usfirst.frc.team1731.robot.ControlBoardInterface;
import org.usfirst.frc.team1731.robot.GamepadControlBoard;
import org.usfirst.frc.team1731.robot.Kinematics;
import org.usfirst.frc.team1731.robot.RobotState;
import org.usfirst.frc.team1731.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    //#region Vision Camera Variables
    boolean visionCamAvailable = false;
    double visionCamXPosition;

    public boolean GetVisionCamAvailable(){
        return visionCamAvailable;
    }

    public double GetVisionCamXPosition(){
        return visionCamXPosition;
    }
    //#endregion


    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
        AttemptVisionCamConnection();
    }

    private ControlBoardInterface mControlBoard = GamepadControlBoard.getInstance();
    private SerialPort visionCam;

    private void AttemptVisionCamConnection(){
        visionCam = new SerialPort(115200, SerialPort.Port.kUSB2);
    }

    @Override
    public synchronized void onLoop(double timestamp) {
        //#region Original Estimator Code

        final double left_distance = drive_.getLeftDistanceInches();
        final double right_distance = drive_.getRightDistanceInches();
        final Rotation2d gyro_angle = drive_.getGyroAngle();
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                drive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;

        //#endregion

        //#region Vision Camera

        boolean tracktorDrive = mControlBoard.getTractorDrive();

        if(visionCam == null){
            AttemptVisionCamConnection();
        } else {
            String visionTargetPositions_Raw = visionCam.readString();
            String[] visionTargetPositions = visionTargetPositions_Raw.split(",");
            visionCamAvailable = visionTargetPositions[0].length() > 0 && visionTargetPositions[0] != null;
            SmartDashboard.putString("RawVisionCamData_Raw", visionTargetPositions_Raw);
            visionCamXPosition = Double.valueOf(visionTargetPositions[0]);
        }

        //#endregion
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
