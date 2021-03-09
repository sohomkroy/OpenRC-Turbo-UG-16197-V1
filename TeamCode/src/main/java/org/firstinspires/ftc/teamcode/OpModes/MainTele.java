/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Differential;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.RaisingServo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.StateClass;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.TurretEncoder;

import java.util.List;

@TeleOp(name="Test", group="CompTele")
public class MainTele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors and Servos
    private DcMotorEx differentialMotor1;
    private DcMotorEx differentialMotor2;
    private Servo intakeServo;
    private Servo servoRaiser;

    //State Class
    //Mechanism Classes
    ServoIntake servoIntake = new ServoIntake();
    Differential differential = new Differential();
    Intake intake = new Intake(differential);
    RaisingServo raisingServo = new RaisingServo();

    TurretEncoder turretEncoder = new TurretEncoder();
    Turret turret = new Turret(differential, turretEncoder);

    private double targetAngle;
    private Pose2d myPose;
    private boolean targetingMode = false;

    SampleMecanumDrive drive;

    public boolean withAuto = true;


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        //TODO create static class to store the position
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        turretEncoder.setInitialAngle(90);


        differentialMotor1  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        differentialMotor2 = hardwareMap.get(DcMotorEx.class, "right_differential_drive");
        differentialMotor1.setDirection(DcMotor.Direction.FORWARD);
        differentialMotor2.setDirection(DcMotor.Direction.FORWARD);

        differentialMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        differentialMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        differentialMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        differentialMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        differentialMotor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        differentialMotor2.getMotorType().setAchieveableMaxRPMFraction(1.0);

        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        servoRaiser = hardwareMap.get(Servo.class, "servo_raiser");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        if (!withAuto) {
            turret.defaultStateReset();
            intake.defaultStateReset();
            servoIntake.defaultStateReset();
            raisingServo.defaultStateReset();
        }

        runtime.reset();
        servoIntake.servoBack();
        raisingServo.servoDown();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            drive.update();
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x*1.1,
//                            -gamepad1.right_stick_x
//                    )
//            );

            if (gamepad2.dpad_down) {
                if (StateClass.getTurretMovement() == StateClass.TurretMovement.STOPPED) {
                    turret.startTurret();
                }
                else {
                    turret.stopTurret();
                }
            }
            if (gamepad2.dpad_left) {
                turret.setTurretSlowMode();
            }
            if (gamepad2.dpad_right) {
                turret.setTurretFastMode();
            }

            if (gamepad1.b) {
                intake.intakeIn();
            }
            if (gamepad1.a) {
                intake.intakeOut();
            }
            if (gamepad1.x) {

                intake.intakeStop();
            }

            if (gamepad2.b) {
                raisingServo.servoUp();
            }
            if (gamepad2.a) {
                raisingServo.servoDown();
            }



            myPose = drive.getPoseEstimate();


            updateMechanisms();
            reportTurretTelemetry();
            reportIntakeTelemetry();
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
    public void updateMechanisms() {
        if (servoIntake.wasChanged()) {
            intakeServo.setPosition(servoIntake.getServoPosition());
        }
        servoIntake.checkServoTimer();

        if (raisingServo.wasChanged()) {
            servoRaiser.setPosition(raisingServo.getServoPosition());
        }
        raisingServo.checkServoTimer();

        turretEncoder.setTurretAngle(drive.getTurretEncoderPosition());
        updateTurretTargetAngle();
        turret.setTurretTargetPosition(targetAngle);
        turret.updateTurret();

//        if (stateClass.getTurretMovement() == StateClass.TurretMovement.MOVING) {
//            turretEncoder.setTurretAngle(drive.getTurretEncoderPosition());
//            updateTurretTargetAngle();
//            turret.setTurretTargetPosition(targetAngle);
//            turret.updateTurret();
//        }
//        else {
//            turret.updateTurret();
//            //probably remove this later
//            turretEncoder.setTurretAngle(drive.getTurretEncoderPosition());
//        }

        if (differential.wasChanged()) {
            differentialMotor1.setPower(differential.getMotor1Power());
            differentialMotor2.setPower(differential.getMotor2Power());
            //telemetry.addData("Diffy 1: ", differential.getMotor1Power());
            //telemetry.addData("Diffy 2: ", differential.getMotor2Power());
            //telemetry.addData("Intake Speed", differential.getIntakeSpeed());
            //telemetry.addData("Turret Speed", differential.getTurretSpeed());
            //telemetry.addData("Velo1", differentialMotor1.getVelocity());
            //telemetry.addData("Velo2", differentialMotor2.getVelocity());
        }
    }

    double deltaX, deltaY;

    double targetX;
    double targetY;
    double uncorrectedAngle;

    double turretLowerAngleBound = 0;
    double turretUpperAngleBound = 360;

    public void updateTurretTargetAngle() {
        if (targetingMode) {
            deltaX = myPose.getX() - targetX;
            deltaY = myPose.getY() - targetY;
            uncorrectedAngle = Math.atan2(deltaY, deltaX) - myPose.getHeading();
            if (uncorrectedAngle > turretUpperAngleBound) {
                targetAngle -= 360;
            }
            if (uncorrectedAngle < turretLowerAngleBound) {
                targetAngle += 360;
            }
        }
        else {
            targetAngle+=-.01*gamepad2.left_stick_y;
            targetAngle = Range.clip(targetAngle, turretLowerAngleBound, turretUpperAngleBound);
        }

        telemetry.addData("Turret Target Angle", targetAngle);
        telemetry.addData("Turret Targeted", turret.onTarget());

    }
    public void reportIntakeTelemetry() {
        switch (StateClass.getIntakeState()) {
            case STOPPED:
                telemetry.addData("Intake", "Stop");
                break;
            case IN:
                telemetry.addData("Intake", "IN");
                break;
            case OUT:
                telemetry.addData("Intake", "OUT");
                break;
        }

    }
    public void reportTurretTelemetry() {
        switch (StateClass.getTurretMovement()) {
            case STOPPED:
                telemetry.addData("Turret:", "Stopped");
                break;
            case MOVING:
                telemetry.addData("Turret:", "Moving");
                break;
        }
        switch (StateClass.getTurretMovementSpeed()) {
            case HIGHPOWER:
                telemetry.addData("Turret Speed", "High Power");
                break;
            case LOWPOWER:
                telemetry.addData("Turret Speed", "Low Power");
                break;
        }
        telemetry.addData("Turret Angle", turretEncoder.getTurretAngle());
        telemetry.addData("Turret Target Angle", targetAngle);

    }
    public void reportServoTelemetry() {

    }
}
