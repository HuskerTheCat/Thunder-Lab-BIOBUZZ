package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Starts facing the goal, launches the preloaded pollen at low flywheel speed, then drives forward and right.
 *
 * Configure during INIT with the dpad (either gamepad):
 *   dpad up/down    = pick a setting
 *   dpad left/right = change it
 *
 * Settings:
 *   Start delay   - extra seconds of launching (it doesn't move before launching, so the delay is spent shooting)
 *   Path          - forward then right, or forward only
 *   Forward       - short or long forward move
 *   Start spot    - center, or side (about a foot left and turned toward the goal so robots in front aren't in the way)
 *
 * All distances, angles and times are placeholders to tune on the field.
 */
@Autonomous(name = "Peak Launch Auto", group = "Auto", preselectTeleOp = "PEAK SCRIMMAGE TELE")
public class Peak_Launch_Auto extends LinearOpMode {

    DcMotor LeftFront;
    DcMotor LeftRear;
    DcMotor RightRear;
    DcMotor RightFront;
    DcMotorEx Flywheel;
    CRServo FeedTop;
    CRServo FeedBottom;
    Servo LaunchServo;

    /* ================= LAUNCHER (keep in sync with Peak_Scrimage_Tele) ================= */
    final double LaunchServoUpPos = 0.00;
    final double LaunchServoDownPos = 0.22;
    final double FeedPower = 0.8;
    final double LOAD_TIME = 1.4;   // sec arm sits down while feeders load a ball
    final double UP_TIME = 0.2;     // sec arm sits up (lift + launch)
    final double FLYWHEEL_LOW = 1800;   // low speed makes the shots, ticks per second
    final double FLYWHEEL_TOLERANCE = 60;
    final double FLYWHEEL_MAX_TPS = 2550;
    // if the flywheel never reads ready, launch anyway after this long so the auto doesn't waste time
    final double MAX_SPINUP_WAIT = 2.0;

    /* ================= TIMING ================= */
    final double AUTO_LENGTH_SEC = 30;
    // feeding isn't consistent, so launch for a long time instead of exactly 4 cycles. Extra cycles just fire empty.
    final double BASE_LAUNCH_SEC = 12;
    // time saved at the end for driving, launching stops early if it would eat into this
    final double DRIVE_RESERVE_SEC = 8;
    final int MAX_START_DELAY_SEC = 10;

    /* ================= DRIVING (placeholders, tune on the field) ================= */
    final double TICKS_PER_INCH = 41.8;          // forward/back
    final double STRAFE_TICKS_PER_INCH = 46.0;   // mecanum strafing slips, so it takes more ticks per inch
    final double TICKS_PER_DEGREE = 8.0;         // turning in place
    final double DRIVE_POWER = 0.5;
    final double SHORT_FORWARD_IN = 12;
    final double LONG_FORWARD_IN = 24;
    final double RIGHT_IN = 24;
    // side start: how far left of the center spot it starts, and how far it's turned toward the goal (clockwise)
    final double SIDE_START_OFFSET_IN = 12;
    final double SIDE_START_ANGLE_DEG = 15;
    // the tele negates strafe and rotation to get the right physical direction, the auto matches it.
    // if the robot strafes or turns the wrong way, flip these to 1
    final int STRAFE_SIGN = -1;
    final int TURN_SIGN = -1;

    /* ================= CONFIG (set during init) ================= */
    int startDelaySec = 0;
    boolean forwardOnly = false;
    boolean longForward = false;
    boolean sideStart = false;

    ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        InitializeHardware();
        ConfigureDuringInit();

        waitForStart();
        autoTimer.reset();
        if (!opModeIsActive()) return;

        // ---- launch ----
        // the start delay is spent launching, capped so there's always time left to drive
        double launchSec = Math.min(BASE_LAUNCH_SEC + startDelaySec,
                AUTO_LENGTH_SEC - DRIVE_RESERVE_SEC - autoTimer.seconds());
        LaunchFor(launchSec);

        // ---- drive ----
        if (sideStart) {
            // turn back from facing the goal to straight before driving
            Drive(0, 0, -SIDE_START_ANGLE_DEG, 6, "Turn back to straight");
        }
        Drive(longForward ? LONG_FORWARD_IN : SHORT_FORWARD_IN, 0, 0, 4, "Forward");
        if (!forwardOnly) {
            // starting a foot left means a foot further to go right
            Drive(0, RIGHT_IN + (sideStart ? SIDE_START_OFFSET_IN : 0), 0, 5, "Right");
        }

        StopEverything();
        while (opModeIsActive()) {
            telemetry.addLine("Done");
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }
    }

    private void ConfigureDuringInit() {
        final String[] names = {"Start delay", "Path", "Forward", "Start spot"};
        int selected = 0;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
            boolean right = gamepad1.dpad_right || gamepad2.dpad_right;

            // only react to new presses so holding a button doesn't race through the options
            if (up && !lastUp) selected = (selected + names.length - 1) % names.length;
            if (down && !lastDown) selected = (selected + 1) % names.length;
            int change = (right && !lastRight) ? 1 : (left && !lastLeft) ? -1 : 0;
            if (change != 0) {
                switch (selected) {
                    case 0: startDelaySec = Range.clip(startDelaySec + change, 0, MAX_START_DELAY_SEC); break;
                    case 1: forwardOnly = !forwardOnly; break;
                    case 2: longForward = !longForward; break;
                    case 3: sideStart = !sideStart; break;
                }
            }
            lastUp = up; lastDown = down; lastLeft = left; lastRight = right;

            String[] values = {
                    startDelaySec + " sec (launches for " + (int) Math.min(BASE_LAUNCH_SEC + startDelaySec,
                            AUTO_LENGTH_SEC - DRIVE_RESERVE_SEC) + " sec)",
                    forwardOnly ? "Forward only" : "Forward then right",
                    longForward ? "Long" : "Short",
                    sideStart ? "Side (1 ft left, turned to goal)" : "Center"
            };
            telemetry.addLine("Dpad up/down = pick, left/right = change");
            for (int i = 0; i < names.length; i++) {
                telemetry.addLine((i == selected ? "> " : "   ") + names[i] + ": " + values[i]);
            }
            telemetry.update();
        }
    }

    private void LaunchFor(double seconds) {
        ElapsedTime phaseTimer = new ElapsedTime();
        ElapsedTime cycleTimer = new ElapsedTime();
        boolean armUp = false;
        int launches = 0;

        Flywheel.setVelocity(FLYWHEEL_LOW);
        while (opModeIsActive() && phaseTimer.seconds() < seconds) {
            double velocity = Flywheel.getVelocity();
            boolean ready = Math.abs(velocity - FLYWHEEL_LOW) < FLYWHEEL_TOLERANCE;
            double feed;

            // same cycle as the tele's launch sequence
            if (!armUp) {
                LaunchServo.setPosition(LaunchServoDownPos);
                feed = cycleTimer.seconds() > LOAD_TIME / 3 ? FeedPower : 0;
                boolean waitedTooLong = cycleTimer.seconds() >= LOAD_TIME + MAX_SPINUP_WAIT;
                if (cycleTimer.seconds() >= LOAD_TIME && (ready || waitedTooLong)) {
                    armUp = true;
                    launches++;
                    cycleTimer.reset();
                }
            } else {
                LaunchServo.setPosition(LaunchServoUpPos);
                feed = cycleTimer.seconds() < UP_TIME * 2 / 3 ? FeedPower : 0;
                if (cycleTimer.seconds() >= UP_TIME) {
                    armUp = false;
                    cycleTimer.reset();
                }
            }
            FeedTop.setPower(feed);
            FeedBottom.setPower(feed);

            telemetry.addData("Phase", "Launching (%.1f / %.1f sec)", phaseTimer.seconds(), seconds);
            telemetry.addData("Launch cycles", launches);
            telemetry.addData("Flywheel", "%.0f / %.0f %s", velocity, FLYWHEEL_LOW, ready ? "READY" : "");
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }

        LaunchServo.setPosition(LaunchServoDownPos);
        FeedTop.setPower(0);
        FeedBottom.setPower(0);
        Flywheel.setVelocity(0);
    }

    /**
     * Drives a set distance with encoders. Positive forward = forward, positive right = right,
     * positive clockwise = turn clockwise (all as the robot sees it).
     */
    private void Drive(double forwardIn, double rightIn, double clockwiseDeg, double timeoutSec, String label) {
        if (!opModeIsActive()) return;
        int f = (int) Math.round(forwardIn * TICKS_PER_INCH);
        int r = (int) Math.round(rightIn * STRAFE_TICKS_PER_INCH) * STRAFE_SIGN;
        int c = (int) Math.round(clockwiseDeg * TICKS_PER_DEGREE) * TURN_SIGN;

        // same mecanum mix as the tele's MecanumDrive
        LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + f + r + c);
        LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + (f - r) + c);
        RightFront.setTargetPosition(RightFront.getCurrentPosition() + (f - r) - c);
        RightRear.setTargetPosition(RightRear.getCurrentPosition() + (f + r) - c);

        for (DcMotor m : new DcMotor[]{LeftFront, LeftRear, RightFront, RightRear}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(DRIVE_POWER);
        }

        // wait for every wheel, with a timeout so a stuck wheel can't burn the rest of auto
        ElapsedTime moveTimer = new ElapsedTime();
        while (opModeIsActive() && moveTimer.seconds() < timeoutSec
                && (LeftFront.isBusy() || LeftRear.isBusy() || RightFront.isBusy() || RightRear.isBusy())) {
            telemetry.addData("Phase", label);
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }

        for (DcMotor m : new DcMotor[]{LeftFront, LeftRear, RightFront, RightRear}) {
            m.setPower(0);
        }
    }

    private void StopEverything() {
        for (DcMotor m : new DcMotor[]{LeftFront, LeftRear, RightFront, RightRear}) {
            m.setPower(0);
        }
        Flywheel.setVelocity(0);
        FeedTop.setPower(0);
        FeedBottom.setPower(0);
    }

    private void InitializeHardware() {
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        FeedTop = hardwareMap.get(CRServo.class, "TopFeed");
        FeedBottom = hardwareMap.get(CRServo.class, "BottomFeed");
        LaunchServo = hardwareMap.get(Servo.class, "launcherServo");

        // same directions as Peak_Scrimage_Tele
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{LeftFront, LeftRear, RightFront, RightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // same velocity tuning as the tele's low speed
        double F = 32767.0 / FLYWHEEL_MAX_TPS;
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(10.0, 1.0, 0.0, F));
    }
}
