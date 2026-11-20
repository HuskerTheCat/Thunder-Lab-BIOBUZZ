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
 * Starts at C1 facing the goal (up the field), launches the preloaded pollen at low flywheel speed, then drives:
 *   C1 -> diagonal up-left to about A/B2 -> straight up to about A/B3 -> up to the park box at A5
 * The robot keeps facing up the whole time, mecanum just slides diagonally.
 * The field is rotated (not mirrored) between alliances, so the same path works for red and blue.
 *
 * Configure during INIT with the dpad (either gamepad):
 *   dpad up/down    = pick a setting
 *   dpad left/right = change it
 *
 * Settings:
 *   Start delay   - extra seconds of launching (it doesn't move before launching, so the delay is spent shooting)
 *   End           - park at A5, or stop at A3 so it stays out of a partner's way
 *   Start spot    - center of C1, or side (about a foot left and turned toward the goal)
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

    /* ================= PATH (inches, placeholders from the field drawing, 1 tile = 24 in) ================= */
    // leg 1: C1 start diagonally up-left to about A/B2
    final double LEG1_LEFT_IN = 39;
    final double LEG1_FORWARD_IN = 31;
    // leg 2: straight up to about A/B3
    final double LEG2_FORWARD_IN = 16;
    // leg 3: up and a little left to the park box at A5
    final double LEG3_LEFT_IN = 7.5;
    final double LEG3_FORWARD_IN = 52;
    // side start: how far left of the center spot it starts, and how far it's turned toward the goal (clockwise)
    final double SIDE_START_OFFSET_IN = 12;
    final double SIDE_START_ANGLE_DEG = 15;

    /* ================= DRIVING (placeholders, tune on the field) ================= */
    final double TICKS_PER_INCH = 41.8;          // forward/back
    final double STRAFE_TICKS_PER_INCH = 46.0;   // mecanum strafing slips, so it takes more ticks per inch
    final double TICKS_PER_DEGREE = 8.0;         // turning in place
    final double DRIVE_POWER = 0.5;

    /* ================= CONFIG (set during init) ================= */
    int startDelaySec = 0;
    boolean stopAtA3 = false;
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
            // turn back from facing the goal to straight up the field before driving
            Drive(0, 0, -SIDE_START_ANGLE_DEG, 3, "Turn back to straight");
        }
        // starting a foot left means a foot less to go left on the first leg
        Drive(LEG1_FORWARD_IN, -(LEG1_LEFT_IN - (sideStart ? SIDE_START_OFFSET_IN : 0)), 0, 5, "Leg 1: up-left to A2");
        Drive(LEG2_FORWARD_IN, 0, 0, 3, "Leg 2: up to A3");
        if (!stopAtA3) {
            Drive(LEG3_FORWARD_IN, -LEG3_LEFT_IN, 0, 5, "Leg 3: up to park at A5");
        }

        StopEverything();
        while (opModeIsActive()) {
            telemetry.addLine("Done");
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }
    }

    private void ConfigureDuringInit() {
        final String[] names = {"Start delay", "End", "Start spot"};
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
                    case 1: stopAtA3 = !stopAtA3; break;
                    case 2: sideStart = !sideStart; break;
                }
            }
            lastUp = up; lastDown = down; lastLeft = left; lastRight = right;

            String[] values = {
                    startDelaySec + " sec (launches for " + (int) Math.min(BASE_LAUNCH_SEC + startDelaySec,
                            AUTO_LENGTH_SEC - DRIVE_RESERVE_SEC) + " sec)",
                    stopAtA3 ? "Stop at A3" : "Park at A5",
                    sideStart ? "Side (1 ft left, turned to goal)" : "Center of C1"
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
     * Drives a set distance. Positive forward = forward, positive right = right,
     * positive clockwise = turn clockwise (all as the robot sees it).
     *
     * Wheels are driven with plain power like the tele (see InitializeHardware for the swapped front motor names),
     * and the encoders only measure how far it's gone. It doesn't use RUN_TO_POSITION: on a motor whose
     * encoder counts backwards from its power, RUN_TO_POSITION "corrects" the wrong way and runs away,
     * which spins the robot. Measuring distance with absolute values doesn't care which way an encoder counts.
     */
    private void Drive(double forwardIn, double rightIn, double clockwiseDeg, double timeoutSec, String label) {
        if (!opModeIsActive()) return;
        double f = forwardIn * TICKS_PER_INCH;
        double r = rightIn * STRAFE_TICKS_PER_INCH;
        double c = clockwiseDeg * TICKS_PER_DEGREE;

        // standard mecanum mix by PHYSICAL wheel position, in ticks each wheel needs to travel
        DcMotor[] wheels = {LeftFront, LeftRear, RightFront, RightRear};
        double[] wheelTicks = {f + r + c, (f - r) + c, (f - r) - c, (f + r) - c};

        double totalTicks = 0;
        double biggest = 0;
        for (double t : wheelTicks) {
            totalTicks += Math.abs(t);
            biggest = Math.max(biggest, Math.abs(t));
        }
        if (totalTicks < 1) return;

        int[] start = new int[4];
        for (int i = 0; i < 4; i++) start[i] = wheels[i].getCurrentPosition();

        ElapsedTime moveTimer = new ElapsedTime();
        double progress = 0;
        while (opModeIsActive() && moveTimer.seconds() < timeoutSec && progress < 1) {
            // progress = how much total wheel travel is done out of what's needed
            double traveled = 0;
            int[] moved = new int[4];
            for (int i = 0; i < 4; i++) {
                moved[i] = wheels[i].getCurrentPosition() - start[i];
                traveled += Math.abs(moved[i]);
            }
            progress = traveled / totalTicks;

            // slow down over the last part so it doesn't coast past the target
            double power = DRIVE_POWER * Range.clip((1 - progress) * 3, 0.3, 1);
            for (int i = 0; i < 4; i++) {
                // each wheel's power is proportional to how far it has to go, so diagonals stay straight
                wheels[i].setPower(power * wheelTicks[i] / biggest);
            }

            telemetry.addData("Phase", label);
            telemetry.addData("Progress", "%.0f%%", progress * 100);
            // if one wheel's number stays near 0 or runs way ahead of the others, that wheel or its encoder cable is the problem
            telemetry.addData("Encoders LF/LR/RF/RR", "%d / %d / %d / %d", moved[0], moved[1], moved[2], moved[3]);
            telemetry.addData("Time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }

        for (DcMotor m : wheels) m.setPower(0);
        sleep(150); // let it settle before the next move
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
        // The robot configuration has the two FRONT motor names swapped: the motor named "RightFront"
        // (port 0) is physically the left front wheel, and "LeftFront" (port 1) is the right front.
        // The tele works around this with its stick swaps and negations. Here each variable is the
        // wheel's real physical position, so the drive math above is the normal mecanum mix.
        LeftFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        FeedTop = hardwareMap.get(CRServo.class, "TopFeed");
        FeedBottom = hardwareMap.get(CRServo.class, "BottomFeed");
        LaunchServo = hardwareMap.get(Servo.class, "launcherServo");

        // same per-motor directions as Peak_Scrimage_Tele, which works out to both left wheels REVERSE
        // and both right wheels FORWARD by physical position
        LeftFront.setDirection(DcMotor.Direction.REVERSE);   // config name "RightFront"
        RightFront.setDirection(DcMotor.Direction.FORWARD);  // config name "LeftFront"
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{LeftFront, LeftRear, RightFront, RightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // plain power like the tele, encoders are only read for distance
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
