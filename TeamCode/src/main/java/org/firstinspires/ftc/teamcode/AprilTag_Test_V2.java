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
 * 1. Sign conventions      - the ground truth every formula below is built on
 * 2. Constants              - tuning numbers for tracking, searching, and the camera
 * 3. Mode                   - the four things the camera motor can be doing
 * 4. RateTracker            - helper: feed it values, get back a rate of change
 * 5. Variables              - camera/AprilTag/motor/IMU objects and state
 * 6. runOpMode()
 *    a. Setup               - camera, Camera_Motor, IMU, fixed exposure
 *    b. Wait for start
 *    c. Main loop           - tag detection, mode selection, motor control
 *    d. Shutdown
 * 7. pickMode()             - decides TRACKING / SEARCHING / MANUAL_OVERRIDE / IDLE
 * 8. trackingPower()        - PD + IMU feedforward toward a visible tag
 * 9. searchingPower()       - PD toward the last known heading, IMU-corrected
 * 10. rampTowards()         - eases motor power toward a target
 * 11. normalizeAngle()      - keeps an angle in -180 to +180
 * 12. setManualExposure()   - locks the camera to a fast, fixed exposure
 */

// Sees AprilTags/clusters and turns Camera_Motor to keep the camera on them.
// Auto-tracks by default; push the left stick to take over manually.
//
// This is a clean rebuild of AprilTag_Test with the same tuned behavior, but
// reorganized: one documented set of sign conventions instead of them being
// discovered piecemeal, a small helper instead of copy-pasted PD bookkeeping,
// and an explicit mode instead of nested if/else guessing what state we're in.
@TeleOp(name = "AprilTag Test V2", group = "Test")
public class AprilTag_Test_V2 extends LinearOpMode {

    // ===== 1. Sign conventions =====
    // All confirmed by physically testing this robot - if the hardware changes
    // (motor direction, mounting), re-verify these before trusting the formulas below.
    //   - Positive bearing (from AprilTag library) = tag is LEFT of camera center.
    //   - Turning the camera LEFT increases cameraDegrees (see below).
    //   - Negative motor power turns the camera LEFT.
    //   - So: to chase a positive bearing (tag is left), power must go negative.
    //     Every power formula here is negated for that reason.

    // ===== 2. Constants =====
    private static final double TICKS_PER_REVOLUTION = 1425.1; // goBILDA 117 RPM motor
    private static final double TEST_POWER_LIMIT = 0.3;
    private static final double STICK_OVERRIDE_DEADZONE = 0.05; // ignore stick drift/noise

    private static final double TRACK_GAIN = 0.03;
    private static final double TRACK_GAIN_D = 0.0018; // keep ~1:17 ratio to TRACK_GAIN
    private static final double TRACK_FEEDFORWARD_GAIN = 0.01; // reacts to IMU spin before bearing error grows
    private static final double MAX_TRACK_POWER = 0.5;
    private static final double TRACK_DEADBAND_DEGREES = 2.0;
    private static final double MAX_POWER_CHANGE_PER_LOOP = 0.03;

    private static final double SEARCH_TIMEOUT_MS = 1500;
    private static final double SEARCH_POWER = 0.8;
    private static final double SEARCH_GAIN = 0.03;
    private static final double SEARCH_GAIN_D = 0.001;
    private static final double SEARCH_POWER_CHANGE_PER_LOOP = 0.08; // ramps up faster than tracking does
    private static final double SEARCH_BRAKE_ZONE_DEGREES = 10.0;
    private static final double SEARCH_BRAKE_POWER = 0.2;

    // ===== Camera =====
    private static final int CAMERA_EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;

    // Ignore small camera jitters from movement
    private static final double RATE_NOISE_FLOOR_DEGREES = 0.3;

    // ===== 3. Mode =====
    private enum Mode { TRACKING, SEARCHING, MANUAL_OVERRIDE, IDLE }

    // ===== 4. RateTracker =====
    private static class RateTracker {
        private Double lastValue = null;
        private final ElapsedTime timer = new ElapsedTime();

        double update(double value) {
            double rate = 0;
            if (lastValue != null && timer.seconds() > 0) {
                double delta = value - lastValue;
                if (Math.abs(delta) > RATE_NOISE_FLOOR_DEGREES) {
                    rate = delta / timer.seconds();
                }
            }
            lastValue = value;
            timer.reset();
            return rate;
        }

        void reset() {
            lastValue = null;
        }
    }

    // ===== 5. Variables =====
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx cameraMotor;
    private IMU imu;
    private double lastMotorPower = 0;

    // Camera angle + robot yaw for the IMU-directed search.
    private Double lastCameraTargetDegrees = null;
    private Double lastRobotYawDegrees = null;
    private final ElapsedTime timeSinceLastSeen = new ElapsedTime();

    private final RateTracker bearingRateTracker = new RateTracker();
    private final RateTracker searchErrorRateTracker = new RateTracker();

    // ===== 6. runOpMode() =====
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

        // IMU orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));

        // ---- b. Wait for start ----
        waitForStart();

        // ---- c. Main loop ----
        while (opModeIsActive()) {

            double cameraDegrees = -(cameraMotor.getCurrentPosition() / TICKS_PER_REVOLUTION * 360.0);
            double robotYawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double yawRateDegPerSec = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            double stickInput = gamepad1.left_stick_x;

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

            if (trackedBearing != null) {
                timeSinceLastSeen.reset();
                lastCameraTargetDegrees = cameraDegrees + trackedBearing;
                lastRobotYawDegrees = robotYawDegrees;
            } else {
                bearingRateTracker.reset();
            }

            boolean lostSightRecently = lastCameraTargetDegrees != null && timeSinceLastSeen.milliseconds() < SEARCH_TIMEOUT_MS;
            Mode mode = pickMode(stickInput, trackedBearing, lostSightRecently);

            double targetPower;
            String trackStatus;

            switch (mode) {
                case MANUAL_OVERRIDE:
                    targetPower = stickInput * TEST_POWER_LIMIT;
                    trackStatus = "manual override";
                    searchErrorRateTracker.reset();
                    break;

                case TRACKING:
                    targetPower = trackingPower(trackedBearing, yawRateDegPerSec);
                    trackStatus = Math.abs(targetPower) < 1e-6
                            ? String.format("bearing %.1f deg, centered", trackedBearing)
                            : String.format("bearing %.1f deg, target power %.2f", trackedBearing, targetPower);
                    searchErrorRateTracker.reset();
                    break;

                case SEARCHING:
                    double searchError = normalizeAngle(lastCameraTargetDegrees - normalizeAngle(robotYawDegrees - lastRobotYawDegrees)) - cameraDegrees;
                    targetPower = searchingPower(searchError);
                    trackStatus = String.format("lost sight - steering toward last known heading (%.1f deg to go)", searchError);
                    break;

                default: // IDLE
                    targetPower = 0;
                    trackStatus = "nothing in view, holding still";
                    searchErrorRateTracker.reset();
                    break;
            }

            double powerChangeLimit = mode == Mode.SEARCHING ? SEARCH_POWER_CHANGE_PER_LOOP : MAX_POWER_CHANGE_PER_LOOP;
            if (Math.abs(targetPower) > Math.abs(lastMotorPower)) {
                lastMotorPower = rampTowards(lastMotorPower, targetPower, powerChangeLimit);
            } else {
                lastMotorPower = targetPower;
            }
            cameraMotor.setPower(lastMotorPower);

            telemetry.addData("Mode", mode);
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

    // ===== 7. pickMode() =====
    private Mode pickMode(double stickInput, Double trackedBearing, boolean lostSightRecently) {
        if (Math.abs(stickInput) >= STICK_OVERRIDE_DEADZONE) {
            return Mode.MANUAL_OVERRIDE;
        } else if (trackedBearing != null) {
            return Mode.TRACKING;
        } else if (lostSightRecently) {
            return Mode.SEARCHING;
        } else {
            return Mode.IDLE;
        }
    }

    // ===== 8. trackingPower() =====
    private double trackingPower(double trackedBearing, double yawRateDegPerSec) {
        double bearingRate = bearingRateTracker.update(trackedBearing);

        if (Math.abs(trackedBearing) < TRACK_DEADBAND_DEGREES) {
            return 0;
        }

        // Feedforward
        double feedforwardPower = yawRateDegPerSec * TRACK_FEEDFORWARD_GAIN;
        double power = -(trackedBearing * TRACK_GAIN + bearingRate * TRACK_GAIN_D) + feedforwardPower;
        return Range.clip(power, -MAX_TRACK_POWER, MAX_TRACK_POWER);
    }

    // ===== 9. searchingPower() =====
    private double searchingPower(double searchError) {
        double searchErrorRate = searchErrorRateTracker.update(searchError);
        double power = -(searchError * SEARCH_GAIN + searchErrorRate * SEARCH_GAIN_D);

        // Deceleration
        double powerLimit = Math.abs(searchError) < SEARCH_BRAKE_ZONE_DEGREES ? SEARCH_BRAKE_POWER : SEARCH_POWER;
        return Range.clip(power, -powerLimit, powerLimit);
    }

    // ===== 10. rampTowards() =====
    private double rampTowards(double current, double target, double maxChange) {
        double change = Range.clip(target - current, -maxChange, maxChange);
        return current + change;
    }

    // ===== 11. normalizeAngle() =====
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    // ===== 12. setManualExposure() =====
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
