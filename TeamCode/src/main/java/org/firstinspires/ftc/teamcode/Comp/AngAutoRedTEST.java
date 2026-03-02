package org.firstinspires.ftc.teamcode.Comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.HardwareAngRobot;


@Autonomous(name="AngAutonomousRedTest", group="Comp")
public class AngAutoRedTEST extends LinearOpMode {

    public DcMotor motorFrontLeft, motorFrontRight, motorBackRight, motorBackLeft, motorArm;
    public CRServo servoRotate;


    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5f;
    static final double TURN_SPEED = 0.5;

    @Override


    public void runOpMode() {

        motorBackLeft = hardwareMap.dcMotor.get("motor4");
        motorBackRight = hardwareMap.dcMotor.get("motor2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor1");
        motorFrontRight = hardwareMap.dcMotor.get("motor3");
        motorArm = hardwareMap.dcMotor.get("launchmech");


        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontRight.setTargetPosition(4000);
        motorFrontLeft.setTargetPosition(4000);
        motorBackRight.setTargetPosition(4000);
        motorBackLeft.setTargetPosition(4000);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        motorFrontRight.setPower(0.1);
        motorFrontLeft.setPower(0.1);
        // due to the front motors being a higher rpm them the back motors
        motorBackRight.setPower(0.25);
        motorBackLeft.setPower(0.25);

        while (opModeIsActive() && motorFrontLeft.isBusy() && motorBackLeft.isBusy()) {
            telemetry.addData("Encoder-Foward-Left-end", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Encoder-Back-Left-end", motorBackLeft.getCurrentPosition());
            telemetry.addData("Encoder-Foward-Right-end", motorFrontRight.getCurrentPosition());
            telemetry.addData("Encoder-Back-Right-end", motorBackRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        resetRuntime();

        while (opModeIsActive() && getRuntime() < 5) {
            telemetry.addData("Encoder-Foward-Left-end", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Encoder-Back-Left-end", motorBackLeft.getCurrentPosition());
            telemetry.addData("Encoder-Foward-Right-end", motorFrontRight.getCurrentPosition());
            telemetry.addData("Encoder-Back-Right-end", motorBackRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorFrontRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);

        motorFrontRight.setPower(-0.1);
        motorFrontLeft.setPower(-0.1);
        motorBackRight.setPower(-0.25);
        motorBackLeft.setPower(-0.25);

        resetRuntime();

    }
}