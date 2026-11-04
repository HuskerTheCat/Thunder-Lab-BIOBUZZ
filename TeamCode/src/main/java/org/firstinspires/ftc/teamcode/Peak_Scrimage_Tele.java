package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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
    DcMotorEx Flywheel;
    DcMotor Intake;
    IMU imu;

    //servo positions are 0-1, so convert from degrees using the servo's full range
    final double FlowerIntakeServoRangeDeg = 270;
    final double FlowerIntakeUpPos = 0 / FlowerIntakeServoRangeDeg;
    final double FlowerIntakeDownPos = 120 / FlowerIntakeServoRangeDeg;
    final double flywheelVelocity = 1000; //encoder ticks per second
    final double TurretPower = 0.4;
    final double IntakePower = 0.8;

    //drive speeds, picked with the triggers so the bumpers are free for the turret
    final double SlowThrottle = 0.4;
    final double NormalThrottle = 0.7;
    final double FastThrottle = 1.0;
    double throttle = NormalThrottle;

    boolean flywheelOn = false;
    boolean lastFlywheelButton = false;

    final float TurretGearRatio = 27.0f / 100.0f;
    @Override
    public void runOpMode() {
        InitializeMotors();
        InitalizeIMU();

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.right_trigger>0.5||gamepad2.right_trigger>0.5){
                throttle = FastThrottle;
            }else if(gamepad1.left_trigger>0.5||gamepad2.left_trigger>0.5){
                throttle = SlowThrottle;
            } else {
                throttle = NormalThrottle;
            }


            float deadZone = 0.05f;
            //use the joysticks on either controller to move the robot, with priority going to gamepad 1
            //maybe controller priority can be moved to a function with lookup table per interaction?
            RobotCentricDrive(
                    -(
                            (
                                    Math.abs(
                                            gamepad1.left_stick_y
                                    )>deadZone?gamepad1:gamepad2
                            ).left_stick_y
                    ) * throttle, //forward/backward, stick y is negative when pushed up
                    ((Math.abs(gamepad1.left_stick_x)>deadZone?gamepad1:gamepad2).left_stick_x)*throttle ,//strafe
                    ((Math.abs(gamepad1.right_stick_x)>deadZone?gamepad1:gamepad2).right_stick_x)*throttle);//rotation

            //basic turret rotation with priority on controller 2
            //if both are pressed just stops, eventualy will go with whichever bumper was pressed most recently
            if(gamepad2.right_bumper&&!gamepad2.left_bumper){
                Turret.setPower(TurretPower);
            } else if (gamepad2.left_bumper&&!gamepad2.right_bumper) {
                Turret.setPower(-TurretPower);
            } else if (gamepad1.right_bumper&&!gamepad1.left_bumper) {
                Turret.setPower(TurretPower);
            } else if (gamepad1.left_bumper&&!gamepad1.right_bumper) {
                Turret.setPower(-TurretPower);
            } else {
                Turret.setPower(0);
            }

            //intake/outake, priority on controller
            if(gamepad1.cross||gamepad2.cross){
                Intake.setPower(IntakePower);
            }
            else if(gamepad2.circle||gamepad1.circle){
                Intake.setPower(-IntakePower);
            } else {
                Intake.setPower(0);
            }

            //toggles flywheel between 0 and full speed, only on a new press so holding it doesn't flicker
            boolean flywheelButton = gamepad1.dpad_left||gamepad2.dpad_left;
            if(flywheelButton&&!lastFlywheelButton) {
                flywheelOn = !flywheelOn;
                Flywheel.setVelocity(flywheelOn?flywheelVelocity:0);
            }
            lastFlywheelButton = flywheelButton;

            if(gamepad1.dpad_down||gamepad2.dpad_down){
                FlowerIntake.setPosition(FlowerIntakeDownPos);
            }else if (gamepad1.dpad_up||gamepad2.dpad_up){
                FlowerIntake.setPosition(FlowerIntakeUpPos);
            }

            //launch sequence
            if(gamepad1.square||gamepad2.square){
                //insert launcher feeding code here after the physical design is finalized


            }

            telemetry.addData("Throttle", throttle);
            telemetry.addData("Flywheel", flywheelOn?"On":"Off");
            telemetry.addData("Flywheel Velocity", Flywheel.getVelocity());
            telemetry.addData("Turret Position", Turret.getCurrentPosition());
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

        // Set Drive Motors to Brake on Zero Power
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Let the flywheel coast down instead of slamming to a stop
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Set Drive Motor Directions
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.REVERSE); //check on robot, usually both right motors are FORWARD
        // Motor encoders, setVelocity needs RUN_USING_ENCODER
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void InitalizeIMU(){
        imu = hardwareMap.get(IMU.class, "imu");
        //to add later with April Tag stuff
    }
}
