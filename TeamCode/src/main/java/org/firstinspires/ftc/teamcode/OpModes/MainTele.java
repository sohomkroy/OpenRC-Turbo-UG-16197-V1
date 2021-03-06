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

    //State Class
    StateClass stateClass = new StateClass();
    //Mechanism Classes
    ServoIntake servoIntake = new ServoIntake(stateClass);
    Differential differential = new Differential();
    Intake intake = new Intake(stateClass, differential);

    TurretEncoder turretEncoder = new TurretEncoder();
    Turret turret = new Turret(stateClass, differential, turretEncoder);

    private double targetAngle;
    private Pose2d myPose;
    private boolean targetingMode = false;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        //TODO create static class to store the position
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        turretEncoder.setInitialAngle(90);


        differentialMotor1  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        differentialMotor2 = hardwareMap.get(DcMotorEx.class, "right_differential_drive");

        differentialMotor1.setDirection(DcMotor.Direction.FORWARD);
        differentialMotor2.setDirection(DcMotor.Direction.REVERSE);

        differentialMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        differentialMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        differentialMotor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        differentialMotor2.getMotorType().setAchieveableMaxRPMFraction(1.0);

        intakeServo = hardwareMap.get(Servo.class, "intake_servo");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            drive.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x*1.1,
                            -gamepad1.right_stick_x
                    )
            );

            if (gamepad2.dpad_down) {
                if (stateClass.getTurretMovement() == StateClass.TurretMovement.STOPPED) {
                    telemetry.addData("Turret", "Turret Started");
                    turret.startTurret();
                }
                else {
                    telemetry.addData("Turret", "Turret Stopped");
                    turret.stopTurret();
                }
            }
            if (gamepad2.dpad_left) {
                turret.setTurretSlowMode();
                telemetry.addData("TurretSpeed", "Turret SlowMode");
            }
            if (gamepad2.dpad_right) {
                turret.setTurretFastMode();
                telemetry.addData("TurretSpeed", "Turret FastMode");
            }

            //b forward, x is stop, a reverse
            if (gamepad1.b) {
                    intake.intakeIn();
            }
            if (gamepad1.a) {
                intake.intakeOut();
            }
            if (gamepad1.x) {
                intake.intakeStop();
            }


            myPose = drive.getPoseEstimate();
            updateMechanisms();

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

        if (stateClass.getTurretMovement() != StateClass.TurretMovement.STOPPED) {
            turretEncoder.setTurretAngle(drive.getTurretPosition());
            updateTurretTargetAngle();
            turret.setTurretTargetPosition(targetAngle);
            turret.updateTurret();
        }

        if (differential.wasChanged()) {
            differentialMotor1.setPower(differential.getMotor1Power());
            differentialMotor2.setPower(differential.getMotor2Power());
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
            targetAngle+=-1*gamepad2.left_stick_y;
            targetAngle = 10*Range.clip(targetAngle, turretLowerAngleBound, turretUpperAngleBound);
        }
        telemetry.addData("Turret Target Angle: ", targetAngle);
        telemetry.addData("Turret Targeted: ", turret.onTarget());

    }
}
