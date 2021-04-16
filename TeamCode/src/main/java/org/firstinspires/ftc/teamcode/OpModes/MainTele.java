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
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.mechanisms.StickServo;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.TurretEncoder;
import org.firstinspires.ftc.teamcode.mechanisms.WobbleClaw;
import org.firstinspires.ftc.teamcode.mechanisms.WobbleGoal;

import java.util.List;

@Config
@TeleOp(name="Test Comp", group="CompTele")
public class MainTele extends LinearOpMode {
    public static double highGoalServoPosition = .58;
    public static int shot1Speed = -1350;

    double t1;
    final int TOTAL_CYCLES = 1000;
    public static int shot2Speed = -1350;
    public static int shot3Speed = -1350;
    public boolean fullRev;

    public static double kP = 50;
    public static double kI = 1;
    public static double kD = 12;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors and Servos
    private DcMotorEx differentialMotor1;
    private DcMotorEx differentialMotor2;
    public int ringCount = 5;
    public boolean readied = false;

    private ServoImplEx intakeServo;
    private ServoImplEx servoRaiser;
    public boolean withAuto = false;

    private ServoImplEx wobbleServo;
    private ServoImplEx wobbleClawServo;
    ShooterIndexServo shooterIndexServo = new ShooterIndexServo();
    ShooterAngleServo shooterAngleServo = new ShooterAngleServo();


    //State Class
    //Mechanism Classes
    ServoIntake servoIntake = new ServoIntake();
    Differential differential = new Differential();
    Intake intake = new Intake(differential);
    RaisingServo raisingServo = new RaisingServo();
    WobbleGoal wobbleGoal = new WobbleGoal();
    WobbleClaw wobbleClaw = new WobbleClaw();
    TurretEncoder turretEncoder = new TurretEncoder();
    Turret turret = new Turret(differential, turretEncoder);
    Shooter shooter = new Shooter();
    CountDownTimer shooterIndexDrop = new CountDownTimer();
    StickServo stickServo = new StickServo();

    public int shots = 0;
    public double shooterPIDPower;
    private double targetAngle;
    private Pose2d myPose;
    private boolean targetingMode = true;

    SampleMecanumDrive drive;
    double highGoalX = 72;

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
    private ServoImplEx servoStick;


    double deltaX, deltaY;

    double targetX;
    double targetY;
    double uncorrectedAngle;

    private ServoImplEx shooterAngler;
    double turretLowerAngleBound = -270;
    double turretUpperAngleBound = 100;
    PIDFController shooterController;

    public CountDownTimer popperTimer;

    ElapsedTime timer = new ElapsedTime();
    int cycles = 0;
    double highGoalY = 34.125-4;
    public boolean setToZero = false;
    private ServoImplEx servoRaiser2;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        //TODO create static class to store the position
        setTurretEncoderInitialEncoderPosition();


        differentialMotor1  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        differentialMotor2 = hardwareMap.get(DcMotorEx.class, "right_differential_drive");
        servoStick = hardwareMap.get(ServoImplEx.class, "stick_servo");

        differentialMotor1.setDirection(DcMotor.Direction.FORWARD);
        differentialMotor2.setDirection(DcMotor.Direction.FORWARD);


//        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//// create the controller
//        differentialMotor1Controller = new PIDFController(coeffs);
//        differentialMotor2Controller = new PIDFController(coeffs);

        //differentialMotor1Controller.setOutputBounds(-1, 1);
        //differentialMotor2Controller.setOutputBounds(-1, 1);

        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        shooterController = new PIDFController(coeffs);
        shooterController.setOutputBounds(-1, 0);


        differentialMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        differentialMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        differentialMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        differentialMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        differentialMotor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        differentialMotor2.getMotorType().setAchieveableMaxRPMFraction(1.0);

        intakeServo = hardwareMap.get(ServoImplEx.class, "intake_servo");
        servoRaiser = hardwareMap.get(ServoImplEx.class, "servo_raiser");
        servoRaiser2 = hardwareMap.get(ServoImplEx.class, "servo_raiser_two");
        servoIndexer = hardwareMap.get(ServoImplEx.class, "servo_indexer");
        shooterAngler = hardwareMap.get(ServoImplEx.class, "angle_servo");
        wobbleServo = hardwareMap.get(ServoImplEx.class, "wobble_arm");
        wobbleClawServo = hardwareMap.get(ServoImplEx.class, "wobble_claw");


        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor_1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor_2");

        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        shooterMotor2.getMotorType().setAchieveableMaxRPMFraction(1.0);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //telemetry.addData("Status", "Initialized");
        //telemetry.update();

        //telemetry.addData("kP", kP);
        //telemetry.addData("kI", kI);
        //telemetry.addData("kD", kD);
        telemetry.update();
        waitForStart();

        popperTimer = new CountDownTimer();

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
            drive.setPoseEstimate(new Pose2d(-63, 17.125, Math.toRadians(180)));
            shooterAngleServo.defaultStateRest();
            StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
            shooterIndexServo.servoOut();
            wobbleGoal.defaultStateResetTele();
            wobbleClaw.defaultStateReset();
            stickServo.defaultStateReset();
            turretEncoder.setInitialAngle(0);
        }
        else {
            drive.setPoseEstimate(PoseStorage.currentPose);
            turretEncoder.setInitialAngle(PoseStorage.turretAngle);
            wobbleClaw.servoBack();
            wobbleClawServo.setPosition(wobbleClaw.getServoPosition());
        }

        StateClass.setGameStage(StateClass.GameStage.TELE_OP);

        runtime.reset();
        servoIntake.servoBack();
        raisingServo.servoDown();
        boolean flag = false;
        double flagNum = 0;
        shooter.setShooterSpeed(shot1Speed);
        StateClass.setShootingTarget(StateClass.ShootingTarget.HIGH_GOAL);


        //shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, 0, 3, 0));
        timer.reset();
        shooterAngleServo.setServoPosition(highGoalServoPosition);

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
            //gamepad 2 down, left, right, b, a
            //gamepad 1b, a, x
//
//            if (gamepad2.dpad_down) {
//                if (StateClass.getTurretMovement() == StateClass.TurretMovement.STOPPED) {
//                    turret.startTurret();
//                }
//                else {
//                    turret.stopTurret();
//                }
//            }
//            if (gamepad2.dpad_left) {
//                turret.stopTurret();
//            }
//            if (gamepad2.dpad_right) {
//                turret.startTurret();
//                turret.setTurretFastMode();
//            }


            if (gamepad1.right_trigger >=.2) {
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                raisingServo.servoDown();
                intake.intakeIn();
                //turret.stopTurret();
            }
            if (gamepad1.b) {
                intake.intakeOut();
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                //turret.stopTurret();

            }
            if (gamepad1.left_trigger>=.2) {
                intake.intakeStop();
            }

            if (gamepad1.x) {
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
            }

            if (gamepad2.left_trigger>=.2) {
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
            if (gamepad2.right_bumper) {
                if (StateClass.getShootingSequenceState() != StateClass.ShootingSequence.REVING_UP) {
                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    shooterIndexDrop.setTime(0);
                    if (StateClass.getShootingTarget() == StateClass.ShootingTarget.HIGH_GOAL) {
                        ringCount = 5;
                    }
                    else {
                        ringCount = 2;
                    }
                    shooterController.reset();
                    shots = 0;
                    shooter.setShooterSpeed(shot1Speed);
                    turret.setTurretFastMode();
                    turret.startTurret();

                }
                fullRev = true;

                StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);
            }
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
//                if (StateClass.getShootingSequenceState() != StateClass.ShootingSequence.REVING_UP) {
//                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
//                    shooterIndexDrop.setTime(0);
//                    ringCount = 4;
//                    shooterController.reset();
//                    shots = 0;
//                    shooter.setShooterSpeed(shot1Speed);
//                    turret.setTurretFastMode();
//                    turret.startTurret();
//                    fullRev = false;
//                }
                StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
            }

            if (gamepad2.right_trigger>.2) {
                highGoalY-=.2;
                //stickServo.servoDown();
            }
            if (gamepad2.left_trigger>.2) {
                highGoalY+=.2;
                //stickServo.servoUp();
            }

            if (gamepad2.x) {
                StateClass.setGameStage(StateClass.GameStage.TELE_OP);
                StateClass.setShootingTarget(StateClass.ShootingTarget.LEFT_POWERSHOT);
                shooterAngleServo.setServoPosition(.565);
            }
            if (gamepad2.y) {
                StateClass.setGameStage(StateClass.GameStage.TELE_OP);
                StateClass.setShootingTarget(StateClass.ShootingTarget.MIDDLE_POWERSHOT);
                shooterAngleServo.setServoPosition(.565);
            }
            if (gamepad2.b) {
                StateClass.setGameStage(StateClass.GameStage.TELE_OP);
                StateClass.setShootingTarget(StateClass.ShootingTarget.RIGHT_POWERSHOT);
                shooterAngleServo.setServoPosition(.565);
            }

//            if (gamepad2.a) {
//                StateClass.setGameStage(StateClass.GameStage.ENDGAME);
//                StateClass.setShootingTarget(StateClass.ShootingTarget.RIGHT_POWERSHOT);
//                shooterAngleServo.setServoPosition(.565);
//            }

            if (gamepad2.dpad_left) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 62.875, 0));
            }
            if (gamepad2.dpad_right) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -15.375, 0));
            }

            if (gamepad2.dpad_down) {
                StateClass.setGameStage(StateClass.GameStage.TELE_OP);
                StateClass.setShootingTarget(StateClass.ShootingTarget.HIGH_GOAL);
                shooterAngleServo.setServoPosition(.58);
            }


//            if (gamepad2.x) {
//                wobbleClaw.servoBack();
//            }
//            if (gamepad2.y) {
//                wobbleClaw.servoOpen();
//            }
//            if (gamepad2.b) {
//                wobbleClaw.servoClamped();
//            }

//            if (gamepad1.dpad_right) {
//                if (StateClass.getShooterState() == StateClass.ShooterState.STOPPED) {
//                    StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
//                }
//            }
//            if (gamepad1.dpad_left) {
//                StateClass.setShooterState(StateClass.ShooterState.STOPPED);
//            }

//            if (gamepad1.right_trigger>=.5) {
//                servoIntake.servoUp();
//            }
//            if (gamepad1.left_trigger>=.5) {
//                servoIntake.servoUp();
//            }


            myPose = drive.getPoseEstimate();

            updateMechanisms();
            updateTargetPosition();
            //reportServoTelemetry();
            telemetry.addData("turret power", turret.getControllerOutput());
            reportTurretTelemetry();
            //reportIntakeTelemetry();
            //reportShooterTelemetry();
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            telemetry.addData("DeltaX", deltaX);
            telemetry.addData("DeltaY", deltaY);

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

        }
        t1 = timer.milliseconds() / cycles;
        telemetry.addData("timer per cycle", t1);
        telemetry.update();
    }

    public void calculateShooterInfo(double distance) {
        //replace with some function that does something
        //shooter.setShooterSpeed(.8);
    }

    public void updateMechanisms() {
        //telemetry.addData("shots", shots);

        if (servoStick.getPosition() != stickServo.getServoPosition()) {
            servoStick.setPosition(stickServo.getServoPosition());
        }
        stickServo.checkServoTimer();

        if (servoIndexer.getPosition() != shooterIndexServo.getServoPosition()) {
            servoIndexer.setPosition(shooterIndexServo.getServoPosition());
        }
        shooterIndexServo.checkServoTimer();

        shooter.updateShooterState(shooterMotor1.getVelocity());

        if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.REVING_UP) {
            if (fullRev) {
                intake.intakeStop();
                if (StateClass.getShooterState() != StateClass.ShooterState.ATSPEED) {
                    StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
                }

                if (!readied) {
                    //shooterIndexServo.servoOut();
                }
                if (StateClass.getShooterState() != StateClass.ShooterState.ATSPEED || StateClass.getServoRaiserState() != StateClass.ServoRaiserState.UP) {
                    //shooterIndexServo.servoOut();
                }

                raisingServo.servoUp();
                if (ringCount > 0) {
                    //telemetry.addData("Ring Count", ringCount);
                    if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.UP) {
                        //telemetry.addData("Up", "up");



                            if (StateClass.getIndexReady() == StateClass.IndexReady.INDEX_READY) {
                                if (StateClass.getTurretPositionState() == StateClass.TurretPositionState.ONTARGET) {



                                    if (StateClass.getShooterState() == StateClass.ShooterState.ATSPEED) {
                                        readied = true;

                                        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT) {
                                            shooterIndexServo.servoIn();
                                            ringCount -= 1;
                                            shots += 1;

                                        }

                                    }
                                    if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN) {
                                        shooterIndexServo.servoOut();
                                        if (shots == 1) {
                                            shooter.setShooterSpeed(shot2Speed);
//                                            if (StateClass.getGameStage()== StateClass.GameStage.ENDGAME) {
//                                                StateClass.setShootingTarget(StateClass.ShootingTarget.MIDDLE_POWERSHOT);
//                                            }

                                        }
                                        if (shots == 2) {
                                            shooter.setShooterSpeed(shot3Speed);
//                                            if (StateClass.getGameStage()== StateClass.GameStage.ENDGAME) {
//                                                StateClass.setShootingTarget(StateClass.ShootingTarget.LEFT_POWERSHOT);
//                                            }

                                        }
                                    }

                            }

                        }

                    } else {
                        //telemetry.addData("Ready to shoot", "Servo Raiser is not up");
                    }

                } else {
                    shooterIndexServo.servoOut();
                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                    readied = false;
                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
                    turret.controller.reset();
                    turret.stopTurret();
                }
            }
            else {
                if (StateClass.getShooterState() != StateClass.ShooterState.ATSPEED) {
                    StateClass.setShooterState(StateClass.ShooterState.WINDINGUP);
                }
            }

        }


        if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.NOT_SHOOTING) {
            //raisingServo.servoDown();
            StateClass.setShooterState(StateClass.ShooterState.STOPPED);
            shooterIndexServo.servoOut();
            readied = false;
            StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
            turret.stopTurret();
            turret.stopTurret();
            //turret.controller.reset();
        }

        if (shooterAngler.getPosition() != shooterAngleServo.getServoPosition()) {
            shooterAngler.setPosition(shooterAngleServo.getServoPosition());

        }

        if (intakeServo.getPosition() != servoIntake.getServoPosition()) {
            intakeServo.setPosition(servoIntake.getServoPosition());

        }

        servoIntake.checkServoTimer();

//        if (servoRaiser.getPosition() != raisingServo.getServoPosition()) {
//            servoRaiser.setPosition(raisingServo.getServoPosition());
//
//        }
        servoRaiser.setPosition(raisingServo.getServoPosition1());
        servoRaiser2.setPosition(raisingServo.getServoPosition2());

        raisingServo.checkServoTimer();

        if (StateClass.getWobbleArmState() != StateClass.WobbleArmState.IDLE) {
            if (wobbleServo.getPosition() != wobbleGoal.getServoPosition()) {
                wobbleServo.setPosition(wobbleGoal.getServoPosition());
            }
            wobbleGoal.checkServoTimer();
        }

        if (wobbleClawServo.getPosition() != wobbleClaw.getServoPosition()) {
            wobbleClawServo.setPosition(wobbleClaw.getServoPosition());
        }
        wobbleClaw.checkServoTimer();

        turretEncoder.setTurretAngle(drive.getTurretEncoderPosition());
        updateTurret();
        turret.setTurretTargetPosition(targetAngle);
        turret.updateTurret();

        //shooter.updateShooter(shooterMotor1.getVelocity());

        if (true) {//(StateClass.getShooterState() != StateClass.ShooterState.STOPPED) {
            shooterController.setTargetPosition(-1);
            //telemetry.addData("measured position", -shooterMotor1.getVelocity()/shooter.getShooterSpeed());

            shooterPIDPower = shooterController.update(-shooterMotor1.getVelocity()/shooter.getShooterSpeed());
            //telemetry.addData("PID", "updating pid");
            //shooterPIDPower = 0.5;
            shooterMotor1.setPower(shooterPIDPower);
            shooterMotor2.setPower(shooterPIDPower);
            setToZero = false;

            //shooterMotor1.setVelocity(shooter.getShooterSpeed());
            //shooterMotor2.setPower(shooter.getShooterSpeed()/shooterMotor2.getMotorType().getAchieveableMaxTicksPerSecond());
        }
        else  {
            shooterMotor1.setPower(-.8);
            shooterMotor2.setPower(-.8);
            setToZero = true;
        }


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
        //intake.updateTimer();
//        if (differential.wasChanged()) {
//
////            differentialMotor1Controller.setTargetPosition(differentialMotor1.getMotorType().getAchieveableMaxTicksPerSecond() * differential.getMotor1Power());
////            differentialMotor1.setPower(differentialMotor1Controller.update(differentialMotor1.getVelocity()));
////
////            differentialMotor2Controller.setTargetPosition(differentialMotor2.getMotorType().getAchieveableMaxTicksPerSecond() * differential.getMotor2Power());
////            differentialMotor2.setPower(differentialMotor2Controller.update(differentialMotor2.getVelocity()));
////
//            differentialMotor1.setPower(differential.getMotor1Power());
//            differentialMotor2.setPower(differential.getMotor2Power());
//
////            telemetry.addData("Diffy 1: ", differential.getMotor1Power());
////            telemetry.addData("Diffy 2: ", differential.getMotor2Power());
////            telemetry.addData("Intake Speed", differential.getIntakeSpeed());
////            telemetry.addData("Turret Speed", differential.getTurretSpeed());
////            telemetry.addData("Velo1", differentialMotor1.getVelocity());
////            telemetry.addData("Velo2", differentialMotor2.getVelocity());
//        }
    }

    public void updateTurret() {
        if (targetingMode) {
//            deltaX = myPose.getX() - targetX;
//            deltaY = myPose.getY() - targetY;
//            uncorrectedAngle = Math.atan2(deltaY, deltaX) - myPose.getHeading();
//            uncorrectedAngle = Math.toDegrees(uncorrectedAngle);

            //targetAngle = -Math.toDegrees(myPose.getHeading());
            //targetAngle = Math.toDegrees(Math.atan2((myPose.getY()), (xOffset+myPose.getX()))-myPose.getHeading());



            deltaX = targetX-(myPose.getX()+1.67035433*Math.cos(myPose.getHeading()));
            deltaY = targetY-(myPose.getY()+1.67035433*Math.sin(myPose.getHeading()));

            uncorrectedAngle = Math.atan2(deltaY, deltaX);

            uncorrectedAngle-=myPose.getHeading();

            if (uncorrectedAngle < 0) {
                uncorrectedAngle+=Math.PI*2;
            }

            distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            targetAngle = Math.toDegrees(uncorrectedAngle);

            if (targetAngle > turretUpperAngleBound) {
                targetAngle -= 360;
            }
            if (targetAngle < turretLowerAngleBound) {
                targetAngle += 360;
            }
            //double turretLowerAngleBound = -270;
            //    double turretUpperAngleBound = 100;
            if (targetAngle<-225) {
                targetAngle = -225;
            }

            if (targetAngle>45) {
                targetAngle = 45;
            }


            calculateShooterInfo(distance);

        }
//        else {
//            targetAngle+=-.01*gamepad2.left_stick_y;
//            targetAngle = Range.clip(targetAngle, turretLowerAngleBound, turretUpperAngleBound);
//        }

        telemetry.addData("Turret Target Angle", targetAngle);
        telemetry.addData("Turret Targeted", turret.onTarget());

    }

    public void updateTargetPosition() {
        switch (StateClass.getShootingTarget()) {
            case HIGH_GOAL:
                targetX = highGoalX;
                targetY = highGoalY;
                break;
            case LEFT_POWERSHOT:
                targetX = 72;
                targetY = (22.75-4.25)-10;
                break;
            case MIDDLE_POWERSHOT:
                targetX = 72;
                targetY = (22.75-11.0)-10;
                break;
            case RIGHT_POWERSHOT:
                targetX = 72;
                targetY = (22.75-19.5)-10;
                break;
        }
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
        telemetry.addData("Turret Uncorrected Angle", uncorrectedAngle);

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

        packet.put("Shooter Target Speed", -shooterController.getTargetPosition());
        packet.put("Applied Power", shooterPIDPower);
        packet.put("Shooter Actual Speed", shooterMotor1.getVelocity()/shooter.getShooterSpeed());
        packet.put("ShooterSpeed", -shooterMotor1.getVelocity());
        telemetry.addData("Applied Power", shooterPIDPower);

        telemetry.addData("Shooter Target Speed", shooter.getShooterSpeed());
        telemetry.addData("Shooter Actual Speed", shooterMotor1.getVelocity());
        telemetry.addData("Shooter Percent Error", shooter.getPercentError());
    }

}
