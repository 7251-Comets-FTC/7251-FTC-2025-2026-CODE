package org.firstinspires.ftc.teamcode.Comp;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.teamcode.Hardware.HardwareAngRobot;

@TeleOp(name="AngTeleOp-Revamp", group="Comp")
public class AngTeleOp3 extends LinearOpMode {

    HardwareAngRobot robot = new HardwareAngRobot(this);
    @Override
    public void runOpMode(){

        robot.init();
        telemetry.addData("Status", "Init Complete");

        waitForStart();
//      This is what Actually Controls the Robot, Motors 1-4 are Mechanum Wheel Control and Motor 5 is Launching Wheel.
        while (opModeIsActive()) {




            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x * -1;
            double slowdown = 1;
            double slowdown1 = 1;
            double throttle_control = 0.6;

            if (gamepad1.dpad_up) {
                x = 1.0;
            }

            if (gamepad1.dpad_down) {
                x = -1.0;
            }

            if (gamepad1.dpad_left) {
                y = 1.0;
            }

            if (gamepad1.dpad_right) {
                y = -1.0;
            }
            // used for control of mechanum wheels //
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.right_trigger > 0) {
                slowdown -= 0.5;
            }

            robot.motor1.setPower(frontLeftPower * throttle_control * slowdown);
            robot.motor2.setPower(backLeftPower * throttle_control * slowdown);
            robot.motor3.setPower(frontRightPower * throttle_control * slowdown);
            robot.motor4.setPower(backRightPower * throttle_control * slowdown);

            if (gamepad1.right_bumper) {
                slowdown1 = 0.45;
            }

            else {
                slowdown1 = 1.0;
            }

            if (gamepad1.a) {
                robot.tor5.setPower(1 * slowdown1);
                robot.motor6.setPower(-1 * slowdown1);
            }
            else {
                robot.motor5.setPower(0.0);
                robot.motor6.setPower(0.0);
            }

            if (gamepad1.b) {
                robot.servo1.setPosition(1.5);
            }
            else {
                robot.servo1.setPosition(0.0);
            }



        }



    }

}
