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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModes.Util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.CountDownTimer;
import org.firstinspires.ftc.teamcode.mechanisms.Differential;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.RaisingServo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoIntake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterAngleServo;
import org.firstinspires.ftc.teamcode.mechanisms.ShooterIndexServo;
import org.firstinspires.ftc.teamcode.mechanisms.StateClass;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.TurretEncoder;

import java.util.List;

@Config
@TeleOp(name="Test Comp", group="CompTele")
public class MainTele extends LinearOpMode {

    public static double kP;
    public static double kI;
    public static double kD;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors and Servos
    private DcMotorEx differentialMotor1;
    private DcMotorEx differentialMotor2;
    public int ringCount = 4;
    public boolean readied = false;

    private ServoImplEx intakeServo;
    private ServoImplEx servoRaiser;
    ShooterIndexServo shooterIndexServo = new ShooterIndexServo();
    ShooterAngleServo shooterAngleServo = new ShooterAngleServo();

    //State Class
    //Mechanism Classes
    ServoIntake servoIntake = new ServoIntake();
    Differential differential = new Differential();
    Intake intake = new Intake(differential);
    RaisingServo raisingServo = new RaisingServo();

    TurretEncoder turretEncoder = new TurretEncoder();
    Turret turret = new Turret(differential, turretEncoder);
    Shooter shooter = new Shooter();
    CountDownTimer shooterIndexDrop = new CountDownTimer();
    double turretUpperAngleBound = 90;
    private double targetAngle;
    private Pose2d myPose;
    private boolean targetingMode = true;

    SampleMecanumDrive drive;

    public boolean withAuto = false;

    FtcDashboard dashboard;

    TelemetryPacket packet;
    PIDFController differentialMotor1Controller;
    PIDFController differentialMotor2Controller;
    double xOffset = 2;

    public void setTurretEncoderInitialEncoderPosition() {
        turretEncoder.setInitialTicks(drive.getTurretEncoderPosition());
    }
    double distance = 5;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private ServoImplEx servoIndexer;

    double deltaX, deltaY;

    double targetX;
    double targetY;
    double uncorrectedAngle;

    double turretLowerAngleBound = 0;
    private ServoImplEx shooterAngler;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        //TODO create static class to store the position
        setTurretEncoderInitialEncoderPosition();
        turretEncoder.setInitialAngle(0);


        differentialMotor1  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        differentialMotor2 = hardwareMap.get(DcMotorEx.class, "right_differential_drive");
        differentialMotor1.setDirection(DcMotor.Direction.FORWARD);
        differentialMotor2.setDirection(DcMotor.Direction.FORWARD);

        double kI = differentialMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        double kP = differentialMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
        double kD = differentialMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
//        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//// create the controller
//        differentialMotor1Controller = new PIDFController(coeffs);
//        differentialMotor2Controller = new PIDFController(coeffs);

        //differentialMotor1Controller.setOutputBounds(-1, 1);
        //differentialMotor2Controller.setOutputBounds(-1, 1);


        differentialMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        differentialMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        differentialMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        differentialMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        differentialMotor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        differentialMotor2.getMotorType().setAchieveableMaxRPMFraction(1.0);

        intakeServo = hardwareMap.get(ServoImplEx.class, "intake_servo");
        servoRaiser = hardwareMap.get(ServoImplEx.class, "servo_raiser");
        servoIndexer = hardwareMap.get(ServoImplEx.class, "servo_indexer");
        shooterAngler = hardwareMap.get(ServoImplEx.class, "angle_servo");

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor_1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor_2");

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor1.getMotorType().setAchieveableMaxRPMFraction(.75);
        shooterMotor2.getMotorType().setAchieveableMaxRPMFraction(.75);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

//        kP = shooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
//        kD = shooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
//        kI = shooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;


        if (!withAuto) {
            turret.defaultStateReset();
            intake.defaultStateReset();
            servoIntake.defaultStateReset();
            raisingServo.defaultStateReset();
            shooter.defaultStateReset();
            shooterIndexServo.defaultStateReset();
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
            shooterAngleServo.defaultStateRest();
            StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
            servoIndexer.setPosition(0.65);
        }
        else {
            drive.setPoseEstimate(PoseStorage.currentPose);
        }

        runtime.reset();
        servoIntake.servoBack();
        raisingServo.servoDown();
        boolean flag = false;
        double flagNum = 0;

        while (opModeIsActive()) {
            shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kD, kI, 0));
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
            //gamepad 2 down, left, right, b, a
            //gamepad 1b, a, x

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

            if (gamepad1.right_trigger >=.2) {
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                intake.intakeIn();
            }
            if (gamepad1.b) {
                intake.intakeOut();

            }
            if (gamepad1.left_trigger>=.2) {
                intake.intakeStop();
            }

//            if (gamepad2.right_bumper) {
//                StateClass.setShootingSequenceState(StateClass.ShootingSequence.SHOOTING);
//                ringCount = 3;
//            }

//            if (gamepad2.left_bumper) {
//                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
//            }




//            if (gamepad1.dpad_up) {
//                raisingServo.servoUp();
//                //servoRaiser.setPosition(1);
//                //intakeServo.setPosition(1);
//            }
//            if (gamepad1.dpad_down) {
//                raisingServo.servoDown();
//                //servoRaiser.setPosition(0);
//                //intakeServo.setPosition(0);
//            }

            if (gamepad1.right_bumper) {
                if (StateClass.getShootingSequenceState() != StateClass.ShootingSequence.REVING_UP) {
                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    shooterIndexDrop.setTime(0);
                    ringCount = 4;
                }
                StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);
            }
            if (gamepad1.left_bumper) {
                if (StateClass.getShootingSequenceState() != StateClass.ShootingSequence.REVING_UP) {
                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    shooterIndexDrop.setTime(0);
                    ringCount = 4;
                }
            }

            if (gamepad2.right_bumper) {
                shooterIndexServo.servoIn();
            }
            if (gamepad2.left_bumper) {
                shooterIndexServo.servoOut();
            }



            if (gamepad1.dpad_right) {
                if (StateClass.getShooterState() == StateClass.ShooterState.STOPPED) {
                    StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
                }
            }
            if (gamepad1.dpad_left) {
                StateClass.setShooterState(StateClass.ShooterState.STOPPED);
            }

            if (gamepad1.right_trigger>=.5) {
                servoIntake.servoUp();
            }
            if (gamepad1.left_trigger>=.5) {
                servoIntake.servoUp();
            }


            myPose = drive.getPoseEstimate();

            updateMechanisms();
            reportServoTelemetry();
            //reportTurretTelemetry();
            //reportIntakeTelemetry();
            reportShooterTelemetry();
            //telemetry.addData("x", myPose.getX());
            //telemetry.addData("y", myPose.getY());
            //telemetry.addData("heading", myPose.getHeading());

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

        }
        servoRaiser.setPwmDisable();
    }

    public void updateMechanisms() {

        shooterIndexServo.checkServoTimer();

        //servoIndexer.setPosition(shooterIndexServo.getServoPosition());

        if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.REVING_UP) {
            intake.intakeStop();
            if (StateClass.getShooterState() != StateClass.ShooterState.ATSPEED) {
                StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
            }

//            if (!readied) {
//                shooterIndexServo.servoOut();
//            }
//            if (StateClass.getShooterState() != StateClass.ShooterState.ATSPEED || StateClass.getServoRaiserState() != StateClass.ServoRaiserState.UP) {
//                shooterIndexServo.servoOut();
//            }

            raisingServo.servoUp();
            if (ringCount > 0) {
                if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.UP) {
                    if (StateClass.getShooterState() == StateClass.ShooterState.ATSPEED) {
                        readied = true;

                        if (StateClass.getIndexReady() == StateClass.IndexReady.INDEX_READY) {
                            if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT && shooterIndexDrop.timeElapsed()) {
                                shooterIndexServo.servoIn();
                                ringCount -= 1;
                                servoIndexer.setPosition(.8);

                            }
                            if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN) {
                                shooterIndexServo.servoOut();
                                servoIndexer.setPosition(.65);
                                shooterIndexDrop.setTime(20);


                            }

                        }

                    }

                }
                else {
                    telemetry.addData("Ready to shoot", "Servo Raiser is not up");
                }
            } else {
                servoIndexer.setPosition(.65);
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                readied = false;
                StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
            }
        }


        if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.NOT_SHOOTING) {
            raisingServo.servoDown();
            StateClass.setShooterState(StateClass.ShooterState.STOPPED);
            //shooterIndexServo.servoOut();
            readied = false;
            StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
        }

        if (shooterAngleServo.isChanged()) {
            shooterAngler.setPosition(shooterAngleServo.getServoPosition());
        }
        if (servoIntake.wasChanged()) {
            intakeServo.setPosition(servoIntake.getServoPosition());
        }
        servoIntake.checkServoTimer();

        if (raisingServo.wasChanged()) {
            servoRaiser.setPosition(raisingServo.getServoPosition());
        }
        raisingServo.checkServoTimer();

        turretEncoder.setTurretAngle(drive.getTurretEncoderPosition());
        updateTurret();
        turret.setTurretTargetPosition(targetAngle);
        turret.updateTurret();

        //shooter.updateShooter(shooterMotor1.getVelocity());

        if (StateClass.getShooterState() != StateClass.ShooterState.STOPPED) {
            shooterMotor1.setPower(shooter.getShooterSpeed());
            shooterMotor2.setPower(shooter.getShooterSpeed());
        }
        else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }

        shooter.updateShooterState(shooterMotor1.getVelocity()/shooterMotor1.getMotorType().getAchieveableMaxTicksPerSecond());

//        if (shooterIndexServo.wasChanged()) {
//            servoIndexer.setPosition(shooterIndexServo.getServoPosition());
//        }



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
        intake.updateTimer();
        if (differential.wasChanged()) {

//            differentialMotor1Controller.setTargetPosition(differentialMotor1.getMotorType().getAchieveableMaxTicksPerSecond() * differential.getMotor1Power());
//            differentialMotor1.setPower(differentialMotor1Controller.update(differentialMotor1.getVelocity()));
//
//            differentialMotor2Controller.setTargetPosition(differentialMotor2.getMotorType().getAchieveableMaxTicksPerSecond() * differential.getMotor2Power());
//            differentialMotor2.setPower(differentialMotor2Controller.update(differentialMotor2.getVelocity()));
//
            differentialMotor1.setPower(differential.getMotor1Power());
            differentialMotor2.setPower(differential.getMotor2Power());

//            telemetry.addData("Diffy 1: ", differential.getMotor1Power());
//            telemetry.addData("Diffy 2: ", differential.getMotor2Power());
//            telemetry.addData("Intake Speed", differential.getIntakeSpeed());
//            telemetry.addData("Turret Speed", differential.getTurretSpeed());
//            telemetry.addData("Velo1", differentialMotor1.getVelocity());
//            telemetry.addData("Velo2", differentialMotor2.getVelocity());
        }
    }

    public void updateTurret() {
        if (targetingMode) {
//            deltaX = myPose.getX() - targetX;
//            deltaY = myPose.getY() - targetY;
//            uncorrectedAngle = Math.atan2(deltaY, deltaX) - myPose.getHeading();
//            uncorrectedAngle = Math.toDegrees(uncorrectedAngle);

            targetAngle = -Math.toDegrees(myPose.getHeading());
            //targetAngle = Math.toDegrees(Math.atan2((myPose.getY()), (xOffset+myPose.getX()))-myPose.getHeading());
            if (uncorrectedAngle > turretUpperAngleBound) {
                targetAngle -= 360;
            }
            if (uncorrectedAngle < turretLowerAngleBound) {
                targetAngle += 360;
            }
            distance = 100;
            calculateShooterInfo(distance);

        }
        else {
            targetAngle+=-.01*gamepad2.left_stick_y;
            targetAngle = Range.clip(targetAngle, turretLowerAngleBound, turretUpperAngleBound);
        }

        telemetry.addData("Turret Target Angle", targetAngle);
        telemetry.addData("Turret Targeted", turret.onTarget());

    }

    public void calculateShooterInfo(double distance) {
        //replace with some function that does something
        //shooter.setShooterSpeed(.8);
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
        packet.put("Turret Angle", turretEncoder.getTurretAngle());
        packet.put("Turret Target Angle", targetAngle);
        packet.put("PID Power", differential.getTurretSpeed());


        telemetry.addData("Turret Angle", turretEncoder.getTurretAngle());
        telemetry.addData("Turret Target Angle", targetAngle);

    }
    public void reportServoTelemetry() {
        switch (StateClass.getServoRaiserState()) {
            case UP:
                telemetry.addData("Raiser", "Up");
                break;
            case DOWN:
                telemetry.addData("Raiser", "Down");
                break;
            case MOVING_DOWN:
                telemetry.addData("Raiser", "Moving Down");
                break;
            case MOVING_UP:
                telemetry.addData("Raiser", "Moving Up");
                break;

        }
        switch (StateClass.getShooterServoState()) {
            case IN:
                telemetry.addData("Indexer", "In");
                break;
            case OUT:
                telemetry.addData("Indexer", "Out");
                break;
            case MOVING_IN:
                telemetry.addData("Indexer", "Moving In");
                break;
            case MOVING_OUT:
                telemetry.addData("Indexer", "Moving Out");
                break;
        }
    }

    public void reportShooterTelemetry() {
        switch (StateClass.getShooterState()) {
            case STOPPED:
                telemetry.addData("Shooter", "Stopped");
                break;
            case WINDINGUP:
                telemetry.addData("Shooter", "Winding Up");
                break;
            case ATSPEED:
                telemetry.addData("Shooter", "At Speed");
                break;
        }

        packet.put("Shooter Target Speed", shooter.getShooterSpeed() * shooterMotor1.getMotorType().getAchieveableMaxTicksPerSecond());
        packet.put("Shooter Actual Speed", shooterMotor1.getVelocity());
        telemetry.addData("Shooter Target Speed", shooter.getShooterSpeed() * shooterMotor1.getMotorType().getAchieveableMaxTicksPerSecond());
        telemetry.addData("Shooter Actual Speed", shooterMotor1.getVelocity());
        telemetry.addData("Shooter Percent Error", shooter.getPercentError());
    }

}
