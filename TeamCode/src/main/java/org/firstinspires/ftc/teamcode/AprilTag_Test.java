package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * TABLE OF CONTENTS
 * 1. Constants            - tuning numbers for tracking, searching, and the camera
 * 2. Variables            - camera/AprilTag/motor/IMU objects and PD state
 * 3. runOpMode()
 *    a. Setup             - camera, Camera_Motor, IMU, fixed exposure
 *    b. Wait for start
 *    c. Main loop         - tag detection, auto-track / manual jog / IMU search
 *    d. Shutdown
 * 4. rampTowards()        - eases motor power toward a target
 * 5. normalizeAngle()     - keeps an angle in -180 to +180
 * 6. setManualExposure()  - locks the camera to a fast, fixed exposure
 */

// Sees AprilTags/clusters and turns Camera_Motor to keep the camera on them.
// Auto-tracks by default; push the left stick to take over manually.
@TeleOp(name = "AprilTag Test", group = "Test")
public class AprilTag_Test extends LinearOpMode {

    // ===== 1. Constants =====
    private static final double TICKS_PER_REVOLUTION = 1425.1; // goBILDA 117 RPM motor
    private static final double TEST_POWER_LIMIT = 0.3;
    private static final double STICK_OVERRIDE_DEADZONE = 0.05; // ignore stick drift/noise

    private static final double TRACK_GAIN = 0.03;
    private static final double TRACK_GAIN_D = 0.0018; // keep ~1:17 ratio to TRACK_GAIN
    private static final double TRACK_FEEDFORWARD_GAIN = 0.006; // reacts to IMU spin before bearing error grows
    private static final double MAX_TRACK_POWER = 0.5;
    private static final double TRACK_DEADBAND_DEGREES = 1.5;
    private static final double MAX_POWER_CHANGE_PER_LOOP = 0.04;

    private static final double SEARCH_TIMEOUT_MS = 1500;
    private static final double SEARCH_POWER = 0.7;
    private static final double SEARCH_GAIN = 0.03;
    private static final double SEARCH_GAIN_D = 0.001;

    // Below this, a frame-to-frame change is just camera jitter, not real
    // motion - ignoring it stops small corrections from overreacting to noise.
    private static final double RATE_NOISE_FLOOR_DEGREES = 0.3;

    // Short, fixed exposure - stops camera motion from blurring the tags.
    private static final int CAMERA_EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;

    // ===== 2. Variables =====
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx cameraMotor;
    private IMU imu;
    private double lastMotorPower = 0;

    // Camera angle + robot yaw at the last sighting, for the IMU-directed search.
    private Double lastCameraTargetDegrees = null;
    private Double lastRobotYawDegrees = null;

    // PD state for tracking and for searching.
    private Double lastBearing = null;
    private final ElapsedTime bearingTimer = new ElapsedTime();
    private Double lastSearchError = null;
    private final ElapsedTime searchErrorTimer = new ElapsedTime();

    private final ElapsedTime timeSinceLastSeen = new ElapsedTime();

    // ===== 3. runOpMode() =====
    @Override
    public void runOpMode() {

        // ---- a. Setup ----
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(CAMERA_EXPOSURE_MS, CAMERA_GAIN);

        cameraMotor = hardwareMap.get(DcMotorEx.class, "Camera_Motor");
        cameraMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        cameraMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cameraMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Control Hub mounted: logo FORWARD, USB ports DOWN.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));

        // ---- b. Wait for start ----
        telemetry.addLine("Auto-tracks by default. Left stick = manual override.");
        telemetry.addLine("Y button = reset IMU heading.");
        telemetry.addLine("Hit START once the camera preview looks good");
        telemetry.update();
        waitForStart();

        // ---- c. Main loop ----
        while (opModeIsActive()) {

            if (gamepad1.y) {
                imu.resetYaw();
            }

            double cameraDegrees = -(cameraMotor.getCurrentPosition() / TICKS_PER_REVOLUTION * 360.0);
            double robotYawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double yawRateDegPerSec = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            // ---- AprilTags/clusters ----
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Things seen", detections.size());

            Double trackedBearing = null;

            for (AprilTagDetection detection : detections) {

                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                    if (tag.metadata != null) {
                        telemetry.addLine(String.format("Tag ID %d (%s)", tag.id, tag.metadata.name));
                        telemetry.addLine(String.format("  range %.1f in, bearing %.1f deg", detection.ftcPose.range, detection.ftcPose.bearing));
                        if (trackedBearing == null) {
                            trackedBearing = detection.ftcPose.bearing;
                        }
                    } else {
                        telemetry.addLine(String.format("Tag ID %d - not in the tag library", tag.id));
                    }

                } else if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("Cluster (%s)", cluster.metadata.name));
                    telemetry.addLine(String.format("  %d%% of the cluster's tags found", cluster.percentClusterFound));
                    telemetry.addLine(String.format("  range %.1f in, bearing %.1f deg, roll %.1f deg", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.roll));
                    trackedBearing = detection.ftcPose.bearing; // clusters win over lone tags
                }
            }

            // ---- Camera_Motor ----
            double targetPower;
            String trackStatus;
            double stickInput = gamepad1.left_stick_x;

            if (Math.abs(stickInput) < STICK_OVERRIDE_DEADZONE) {
                if (trackedBearing != null) {
                    timeSinceLastSeen.reset();

                    double bearingRate = 0;
                    if (lastBearing != null && bearingTimer.seconds() > 0) {
                        double bearingDelta = trackedBearing - lastBearing;
                        // Below the noise floor, treat it as jitter, not real
                        // motion - otherwise the rate term amplifies camera noise.
                        if (Math.abs(bearingDelta) > RATE_NOISE_FLOOR_DEGREES) {
                            bearingRate = bearingDelta / bearingTimer.seconds();
                        }
                    }
                    lastBearing = trackedBearing;
                    bearingTimer.reset();
                    lastCameraTargetDegrees = cameraDegrees + trackedBearing;
                    lastRobotYawDegrees = robotYawDegrees;
                    lastSearchError = null;

                    if (Math.abs(trackedBearing) < TRACK_DEADBAND_DEGREES) {
                        targetPower = 0;
                        trackStatus = String.format("bearing %.1f deg, centered", trackedBearing);
                    } else {
                        // Feedforward: spin picked up by the IMU right now, added
                        // in before the bearing error even has a chance to grow.
                        double feedforwardPower = yawRateDegPerSec * TRACK_FEEDFORWARD_GAIN;
                        targetPower = Range.clip(-(trackedBearing * TRACK_GAIN + bearingRate * TRACK_GAIN_D) + feedforwardPower, -MAX_TRACK_POWER, MAX_TRACK_POWER);
                        trackStatus = String.format("bearing %.1f deg, target power %.2f", trackedBearing, targetPower);
                    }
                } else {
                    lastBearing = null;

                    if (lastCameraTargetDegrees != null && timeSinceLastSeen.milliseconds() < SEARCH_TIMEOUT_MS) {
                        // normalizeAngle() handles the IMU heading wrapping
                        // around from +180 to -180 while we weren't looking.
                        double robotTurnedSinceSeen = normalizeAngle(robotYawDegrees - lastRobotYawDegrees);
                        double targetCameraDegrees = lastCameraTargetDegrees - robotTurnedSinceSeen;
                        double searchError = targetCameraDegrees - cameraDegrees;

                        double searchErrorRate = 0;
                        if (lastSearchError != null && searchErrorTimer.seconds() > 0) {
                            searchErrorRate = (searchError - lastSearchError) / searchErrorTimer.seconds();
                        }
                        lastSearchError = searchError;
                        searchErrorTimer.reset();

                        targetPower = Range.clip(-(searchError * SEARCH_GAIN + searchErrorRate * SEARCH_GAIN_D), -SEARCH_POWER, SEARCH_POWER);
                        trackStatus = String.format("lost sight - steering toward last known heading (%.1f deg to go)", searchError);
                    } else {
                        targetPower = 0;
                        trackStatus = "nothing in view, holding still";
                    }
                }
            } else {
                targetPower = stickInput * TEST_POWER_LIMIT;
                trackStatus = "manual override";
            }

            // Ease into speeding up, but let it slow down/reverse instantly
            // (ramping the slow-down caused overshoot).
            if (Math.abs(targetPower) > Math.abs(lastMotorPower)) {
                lastMotorPower = rampTowards(lastMotorPower, targetPower, MAX_POWER_CHANGE_PER_LOOP);
            } else {
                lastMotorPower = targetPower;
            }
            cameraMotor.setPower(lastMotorPower);

            telemetry.addData("Auto-track", trackStatus);
            telemetry.addData("Motor power", "%.2f", lastMotorPower);
            telemetry.addData("Motor turned", "%.1f deg", cameraDegrees);
            telemetry.addData("IMU heading (yaw)", "%.1f deg", robotYawDegrees);

            telemetry.update();
            sleep(20);
        }

        // ---- d. Shutdown ----
        visionPortal.close();
    }

    // ===== 4. rampTowards() =====
    private double rampTowards(double current, double target, double maxChange) {
        double change = Range.clip(target - current, -maxChange, maxChange);
        return current + change;
    }

    // ===== 5. normalizeAngle() =====
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    // ===== 6. setManualExposure() =====
    private void setManualExposure(int exposureMS, int gain) {
        telemetry.addData("Camera", "Waiting for streaming to start");
        telemetry.update();
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);

        telemetry.addData("Camera", "Ready (manual exposure set)");
        telemetry.update();
    }
}
