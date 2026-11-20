package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
@TeleOp(name = "PEAK SCRIMMAGE TELE", group = "Competition")
public class Peak_Scrimage_Tele extends LinearOpMode{

    //hardware is grabbed in runOpMode, hardwareMap is null until then
    DcMotor LeftFront;
    DcMotor LeftRear;
    DcMotor RightRear;
    DcMotor RightFront;
    DcMotor Turret;
    Servo FlowerIntake;
    CRServo FeedTop;

    CRServo FeedBottom;
    Servo LaunchServo;
    DcMotorEx Flywheel;
    DcMotor Intake;
    IMU imu;

    //servo positions are 0-1, so convert from degrees using the servo's full range
    final double FlowerIntakeServoRangeDeg = 270;
    final double FlowerIntakeUpPos = 0.35;
    final double FlowerIntakeDownPos = 1;
    final double LaunchServoRangeDeg = 270;
    final double LaunchServoUpPos = 0.00;
    final double LaunchServoDownPos = 0.22;
    final double FlowerInitStep = 0.005; //servo position change per loop when adjusting during init
    final double TurretPower = 0.4;
    final double IntakePower = 0.5;

    //drive speeds, picked with the triggers so the bumpers are free for the turret
    final double SlowThrottle = 0.4;
    final double NormalThrottle = 0.7;
    final double FastThrottle = 1.0;
    double throttle = NormalThrottle;

    boolean lastFlywheelSpeedButton = false;
    boolean flywheelSpeedHigh = false;
    //int FLYWHEEL_HIGH = 3500;
    //int FLYWHEEL_LOW = 2450;
    //high runs the flywheel at full power (the old 3500 target was really just full power, the motor can't go that fast).
    //FLYWHEEL_HIGH is only the speed it has to be above before the arm will launch on high.
    //lower it if the arm never lifts on high with a weak battery, retune after the heavier flywheel goes on
    int FLYWHEEL_HIGH = 2350;
    //low uses velocity control, makes shots well at 1800
    int FLYWHEEL_LOW = 1800;
    boolean flywheelFullPower = false; //tracks which run mode the flywheel is in so it only switches when needed
    //real top speed with the flywheel loaded on a normal battery (2860 was free spinning on a full battery,
    //which made F too weak and left the flywheel about 200 under target)
    final double FLYWHEEL_MAX_TPS = 2550;
    boolean flywheelOn = false;
    boolean lastFlywheelButton = false;

    final float TurretGearRatio = 27.0f / 100.0f;
    ElapsedTime launchTimer = new ElapsedTime();
    boolean armUp = false;

    double LOAD_TIME = 1.4;   // sec arm sits down while feeders load a ball
    double UP_TIME   = 0.2;   // sec arm sits up (lift + launch)
    double FLYWHEEL_TARGET = FLYWHEEL_LOW; //encoder ticks per second, used by both the dpad toggle and the launch sequence
    double FLYWHEEL_TOLERANCE = 60; //measured velocity wobbles, too tight and the arm never lifts
    boolean controller1IsNormal = false;
    boolean controller2IsNormal = true;
    @Override
    public void runOpMode() {
        InitializeMotors();
        InitalizeIMU();

            //telemetry.addLine("LaunchServo pos: " + LaunchServo.getPosition());
            //telemetry.addLine("FlowerServo pos: " + FlowerIntake.getPosition());
            //telemetry.addLine("turret pos: " + Turret.getCurrentPosition());

            while (opModeInInit()) {

                telemetry.addLine("Controller Profiles");
                telemetry.addLine("Driver: " + (controller1IsNormal?"<NORMAL Centric> ":"<MILES Centric> "));
                if(gamepad1.dpad_left)controller1IsNormal=false;
                if(gamepad1.dpad_right)controller1IsNormal=true;
                telemetry.addLine("Co-Driver: " + (controller2IsNormal?"<NORMAL Centric> ":"<MILES Centric> "));
                if(gamepad2.dpad_left)controller2IsNormal=false;
                if(gamepad2.dpad_right)controller2IsNormal=true;




            telemetry.update();
            }

/*
        double launchInitPos = LaunchServoDownPos;
        LaunchServo.setPosition(launchInitPos);

        while (opModeInInit()) {
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                launchInitPos += gamepad1.dpad_up ? FlowerInitStep : -FlowerInitStep;
                //launchInitPos = Range.clip(launchInitPos, 0, 1);
                LaunchServo.setPosition(launchInitPos);
            }

            telemetry.addLine("LaunchServo pos: " + LaunchServo.getPosition());
            telemetry.addLine("FlowerServo pos: " + FlowerIntake.getPosition());
            telemetry.addLine("turret pos: " + Turret.getCurrentPosition());
            telemetry.update();
        }

 */
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.right_trigger>0.5||gamepad2.right_trigger>0.5){
                throttle = FastThrottle;
            }else if(gamepad1.left_trigger>0.5||gamepad2.left_trigger>0.5){
                throttle = SlowThrottle;
            } else {
                throttle = NormalThrottle;
            }

        if(gamepad2.dpad_up){
            //  gamepad2.
        }

            float deadZone = 0.05f;
            //use the joysticks on either controller to move the robot, with priority going to gamepad 1
            //maybe controller priority can be moved to a function with lookup table per interaction?
            /*
            float forward = controller1IsNormal?gamepad1.right_stick_y
            RobotCentricDrive(
                    -((Math.abs(gamepad1.left_stick_y)>deadZone?gamepad1:gamepad2).left_stick_y) * throttle, //forward/backward, stick y is negative when pushed up
                    -((Math.abs(gamepad1.left_stick_x)>deadZone?gamepad1:gamepad2).left_stick_x)*throttle ,//strafe
                    -((Math.abs(gamepad1.right_stick_x)>deadZone?gamepad1:gamepad2).right_stick_x)*throttle);//rotation
            */
            RobotCentricDrive(
                    -(((Math.abs(gamepad1.left_stick_y)>deadZone?gamepad1:gamepad2).left_stick_y) * throttle), //forward/backward, stick y is negative when pushed up
                    (-(((Math.abs(gamepad1.right_stick_x)>deadZone?gamepad1:gamepad2).right_stick_x)*throttle)),//strafe
                    (-((Math.abs(gamepad1.left_stick_x)>deadZone?gamepad1:gamepad2).left_stick_x)*throttle));//rotation
            //basic turret rotation with priority on controller 2, circle one way and cross the other
            //(moved off the bumpers so they're free for intake)
            //if both are pressed just stops, eventualy will go with whichever button was pressed most recently
            if(gamepad2.circle&&!gamepad2.cross){
                Turret.setPower(TurretPower);
            } else if (gamepad2.cross&&!gamepad2.circle) {
                Turret.setPower(-TurretPower);
            } else if (gamepad1.circle&&!gamepad1.cross) {
                Turret.setPower(TurretPower);
            } else if (gamepad1.cross&&!gamepad1.circle) {
                Turret.setPower(-TurretPower);
            } else {
                Turret.setPower(0);
            }

            // ---- decide intake/feeder power (applied once at the bottom) ----
            // right bumper = intake in, left bumper = outtake
            double intakePower = 0;
            double feedPower = 0;
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakePower = IntakePower;
                feedPower = 0.8;
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakePower = -IntakePower;
            }

// ---- flywheel on/off toggle ----
            boolean flywheelButton = gamepad1.dpad_left || gamepad2.dpad_left;
            if (flywheelButton && !lastFlywheelButton) flywheelOn = !flywheelOn;
            lastFlywheelButton = flywheelButton;

// ---- flywheel high/low toggle ----
            boolean flywheelSpeedButton = gamepad1.dpad_right || gamepad2.dpad_right;
            if (flywheelSpeedButton && !lastFlywheelSpeedButton) {
                flywheelSpeedHigh = !flywheelSpeedHigh;
                FLYWHEEL_TARGET = flywheelSpeedHigh ? FLYWHEEL_HIGH : FLYWHEEL_LOW;
            }
            lastFlywheelSpeedButton = flywheelSpeedButton;

// ---- flower ----
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                FlowerIntake.setPosition(FlowerIntakeDownPos);
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                FlowerIntake.setPosition(FlowerIntakeUpPos);
            }

// ---- launch sequence ----
            boolean launching = gamepad1.square || gamepad2.square;
            double launchPos = LaunchServoDownPos;

            if (launching) {
                if (!armUp) {
                    launchPos = LaunchServoDownPos;
                    feedPower = launchTimer.seconds() > LOAD_TIME / 3 ? 0.8 : 0;
                    // full power on high has no exact target, so it's ready once it's fast enough
                    boolean ready = flywheelSpeedHigh
                            ? Flywheel.getVelocity() >= FLYWHEEL_HIGH
                            : Math.abs(Flywheel.getVelocity() - FLYWHEEL_TARGET) < FLYWHEEL_TOLERANCE;
                    // flywheel only gates the lift, so a velocity dip can't freeze the arm up
                    if (launchTimer.seconds() >= LOAD_TIME && ready) {
                        armUp = true;
                        launchTimer.reset();
                    }
                } else {
                    launchPos = LaunchServoUpPos;
                    feedPower = launchTimer.seconds() < UP_TIME * 2 / 3 ? 0.8 : 0;
                    if (launchTimer.seconds() >= UP_TIME) {
                        armUp = false;
                        launchTimer.reset();
                    }
                }
            } else {
                armUp = false;
                launchTimer.reset();
            }

// ---- apply every output exactly once per loop ----
            boolean flywheelRunning = flywheelOn || launching;
            if (flywheelRunning && flywheelSpeedHigh) {
                // high: straight full power, no velocity PID to wind up against a speed it can't reach
                if (!flywheelFullPower) {
                    Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    flywheelFullPower = true;
                }
                Flywheel.setPower(1.0);
            } else {
                // low and off: velocity control
                if (flywheelFullPower) {
                    Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SetFlywheelPIDF();
                    flywheelFullPower = false;
                }
                Flywheel.setVelocity(flywheelRunning ? FLYWHEEL_TARGET : 0);
            }
            LaunchServo.setPosition(launchPos);
            FeedTop.setPower(feedPower);
            FeedBottom.setPower(feedPower);
            Intake.setPower(intakePower);
            //telemetry.addData("Throttle", throttle);
            telemetry.addData("Flywheel", (flywheelOn?"On":"Off") + (flywheelSpeedHigh?" HIGH (full power)":" LOW"));
            telemetry.addData("Flywheel Velocity", Flywheel.getVelocity());
            //telemetry.addData("Turret Position", Turret.getCurrentPosition());
            telemetry.addData("Flywheel Target", FLYWHEEL_TARGET);
            telemetry.addData("Flywheel ERROR", Flywheel.getVelocity()-FLYWHEEL_TARGET);
            //telemerty.addData();
            telemetry.addData("Flywheel power", Flywheel.getPower());
            telemetry.update();
        }
    }

    private void RobotCentricDrive(double RobotForward, double RobotRight, double RobotClockwise) {

        // ---Copy to any OpMode with Mecanum Drive for Robot-centric Control---
        telemetry.addData("Drive Mode", "Robot Centric");
        telemetry.addData("Robot Forward", Double.parseDouble(JavaUtil.formatNumber(RobotForward, 2)));
        telemetry.addData("Robot Right", Double.parseDouble(JavaUtil.formatNumber(RobotRight, 2)));
        telemetry.addData("Robot Clockwise", Double.parseDouble(JavaUtil.formatNumber(RobotClockwise, 2)));
        MecanumDrive(RobotForward, RobotRight, RobotClockwise);
    }
    private void MecanumDrive(double RobotForward, double RobotRight, double RobotClockwise) {
        /*
         * This function takes power setting inputs for the three robot-centric drive commands
         * and converts them into motor power commands for a Mecanum wheeled robot. It wheels are
         * turning the wrong direction, switch their direction in the InitializeMotors function.
         */
        double MaxInput;

        // ---Copy to any OpMode for Robot-centric Mecanum Wheel Drive---
        // Scale to Avoid Clipping
        // Get the sum of the magnitudes of the robot commands. One of the motor commands will equal this maximum input.
        MaxInput = Math.abs(RobotForward) + Math.abs(RobotRight) + Math.abs(RobotClockwise);
        // If the maximum input is greater than 1, the motor will clip anything greater than 1 when applying power.
        if (MaxInput > 1) {
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotForward = RobotForward / MaxInput;
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotRight = RobotRight / MaxInput;
            // Divide the current input by the maximum input to scale it relative to the other inputs.
            RobotClockwise = RobotClockwise / MaxInput;
        }
        // Transform Robot Commands to Mecanum Motors
        // Left Front Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        LeftFront.setPower(RobotForward + RobotRight + RobotClockwise);
        // Left Rear Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        LeftRear.setPower((RobotForward - RobotRight) + RobotClockwise);
        // Right Front Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        RightFront.setPower((RobotForward - RobotRight) - RobotClockwise);
        // Right Rear Motor
        // Fuse all three command inputs based on Mecanum Drive table.
        RightRear.setPower((RobotForward + RobotRight) - RobotClockwise);
    }
    private void InitializeMotors() {
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        FlowerIntake = hardwareMap.get(Servo.class, "FlowerIntake");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        FeedTop = hardwareMap.get(CRServo.class, "TopFeed");
        FeedBottom = hardwareMap.get(CRServo.class, "BottomFeed");
        LaunchServo = hardwareMap.get(Servo.class, "launcherServo");

        // Set Drive Motors to Brake on Zero Power
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Let the flywheel coast down instead of slamming to a stop
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Set Drive Motor Directions
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.FORWARD); //check on robot, usually both right motors are FORWARD
        // Motor encoders, setVelocity needs RUN_USING_ENCODER
        //Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SetFlywheelPIDF();
    }
    private void SetFlywheelPIDF() {
        // The hub's default velocity PID has no feedforward (F), so it levels off below target
        // with power stuck under 1. F gives the power the target needs up front, P and I clean up the rest.
        // F = 32767 / max ticks per second at full power (measured with Flywheel Max Test).
        // P and I are much stronger than the textbook 0.1*F / 0.1*P, which only crept up to target over 30-40 sec.
        // P = 10 is the hub's own default (already ran stable on this flywheel), I pulls out the last bit in a few sec.
        double F = 32767.0 / FLYWHEEL_MAX_TPS;
        double P = 10.0;
        double I = 1.0;
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, 0.0, F));
    }
    void InitalizeIMU(){
        imu = hardwareMap.get(IMU.class, "imu");
        //to add later with April Tag stuff
    }
}
