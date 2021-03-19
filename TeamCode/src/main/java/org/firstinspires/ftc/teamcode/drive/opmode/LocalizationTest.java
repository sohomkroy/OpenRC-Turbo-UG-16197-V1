package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden org.firstinspires.ftc.teamcode.drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "org/firstinspires/ftc/teamcode/drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        int counter = 0;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        List<Double> wheelPosition = drive.getWheelPositions();

        double initialLeft = wheelPosition.get(0);
        double initialRight = wheelPosition.get(1);
        double initialFront = wheelPosition.get(3);

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        while (!isStopRequested()) {
            counter+=1;
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            wheelPosition = drive.getWheelPositions();
            telemetry.addData("Left Wheel", wheelPosition.get(0)-initialLeft);
            telemetry.addData("Right Wheel", wheelPosition.get(1)-initialRight);
            telemetry.addData("Front Wheel", wheelPosition.get(3)-initialFront);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading()*180/Math.PI);
            telemetry.addData("counter", drive.counter);
            telemetry.addData("lateral distance", StandardTrackingWheelLocalizer.LATERAL_DISTANCE);

            telemetry.update();

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }
}
