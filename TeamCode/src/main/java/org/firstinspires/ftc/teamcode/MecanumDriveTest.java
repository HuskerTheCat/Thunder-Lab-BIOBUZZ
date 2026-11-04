package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive Test", group = "Test")
public class MecanumDriveTest extends LinearOpMode {

    // Flip any of these if a wheel spins the wrong way
    static final DcMotorSimple.Direction LEFT_FRONT_DIR  = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction LEFT_REAR_DIR   = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction RIGHT_FRONT_DIR = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction RIGHT_REAR_DIR  = DcMotorSimple.Direction.FORWARD;

    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    @Override
    public void runOpMode() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "LeftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "RightRear");

        leftFront.setDirection(LEFT_FRONT_DIR);
        leftRear.setDirection(LEFT_REAR_DIR);
        rightFront.setDirection(RIGHT_FRONT_DIR);
        rightRear.setDirection(RIGHT_REAR_DIR);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready. Left stick = drive/strafe, right stick X = rotate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive  = -gamepad1.left_stick_y;  // stick up is negative
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x;

            double lf = drive + strafe + rotate;
            double rf = drive - strafe - rotate;
            double lr = drive - strafe + rotate;
            double rr = drive + strafe - rotate;

            // Scale down so no wheel exceeds full power
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                                                Math.max(Math.abs(lr), Math.abs(rr))));

            leftFront.setPower(lf / max);
            rightFront.setPower(rf / max);
            leftRear.setPower(lr / max);
            rightRear.setPower(rr / max);

            telemetry.addData("Drive/Strafe/Rotate", "%.2f / %.2f / %.2f", drive, strafe, rotate);
            telemetry.addData("LF / RF", "%.2f / %.2f", lf / max, rf / max);
            telemetry.addData("LR / RR", "%.2f / %.2f", lr / max, rr / max);
            telemetry.update();
        }
    }
}
