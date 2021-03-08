package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.util.Encoder;

@TeleOp(group = "Test", name="GeneralTests")
public class TestClass extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
//        Servo testServo1 = hardwareMap.get(Servo.class, "test_servo1");
//        Servo testServo2 = hardwareMap.get(Servo.class, "test_servo2");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests


        waitForStart();

        if (isStopRequested()) return;
//        double servoPosition1 = 0.5;
//        double servoPosition2 = 0.5;
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_rear_drive"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));
        while (opModeIsActive()) {
            telemetry.addData("Encoder1", leftEncoder.getCurrentPosition());
            telemetry.addData("Encoder2", rightEncoder.getCurrentPosition());
            telemetry.addData("Encoder3", frontEncoder.getCurrentPosition());

            telemetry.update();
//            double y1 = -gamepad1.left_stick_y; // Remember, this is reversed!
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            double y2 = -gamepad1.right_stick_y;
//
//            //Wobble = port 5 hub 0
//            //Raiser = port 4 hub 0
//            servoPosition1 += y1*.01;
//            servoPosition1 = Range.clip(servoPosition1, 0, 1);
//            testServo1.setPosition(servoPosition1);
//            telemetry.addData("ServoPositionTestServo1: ", servoPosition1);
//
//            servoPosition2 += y2*.01;
//            servoPosition2 = Range.clip(servoPosition2, 0, 1);
//            testServo2.setPosition(servoPosition2);
//            telemetry.addData("ServoPositionTestServo2: ", servoPosition2);
//            telemetry.update();

        }
    }
}