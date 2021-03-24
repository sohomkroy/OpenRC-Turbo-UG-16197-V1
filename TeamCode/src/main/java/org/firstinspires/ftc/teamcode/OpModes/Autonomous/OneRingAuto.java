package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point1;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point10;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point10Heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point2;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point2Heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point2HeadingV2;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point3;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point4;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point5;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point6;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point6OneRing;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point7;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point7heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point8;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point8Heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point9;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point9Heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point9OneRing;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.point9OneRingHeading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.points6Heading;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.points6OneRingHeading;

@Autonomous
public class OneRingAuto extends LinearOpMode {
    public static int shot1Speed = -1350;
    public static int shot2Speed = -1300;
    public static int shot3Speed = -1300;
    public static double kP = 50;
    public static double kI = 8;
    public static double kD = 0;
    final int TOTAL_CYCLES = 1000;
    public int ringCount = 4;
    public boolean readied = false;
    public int shots = 0;
    public double shooterPIDPower;
    public boolean withAuto = false;
    public CountDownTimer popperTimer;
    public boolean fullRev = true;
    public boolean setToZero = false;
    double t1;
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
    SampleMecanumDrive drive;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    PIDFController differentialMotor1Controller;
    PIDFController differentialMotor2Controller;
    StickServo stickServo = new StickServo();
    double xOffset = 2;
    double distance = 5;
    double deltaX, deltaY;
    double targetX;
    double targetY;
    double uncorrectedAngle;
    double turretLowerAngleBound = -270;
    double turretUpperAngleBound = 100;
    PIDFController shooterController;
    //ElapsedTime timer = new ElapsedTime();
    int cycles = 0;
    State currentState = State.INIT;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Motors and Servos
    private DcMotorEx differentialMotor1;
    private DcMotorEx differentialMotor2;
    private ServoImplEx intakeServo;
    private ServoImplEx servoRaiser;
    private ServoImplEx wobbleServo;
    private ServoImplEx wobbleClawServo;
    private double targetAngle;
    private Pose2d myPose;
    private boolean targetingMode = true;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private ServoImplEx servoIndexer;
    private ServoImplEx shooterAngler;
    private ServoImplEx servoStick;

    public void setTurretEncoderInitialEncoderPosition() {
        turretEncoder.setInitialTicks(drive.getTurretEncoderPosition());
    }

    public boolean needsToShoot = true;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // main trajectories
        Trajectory traj1 = drive.trajectoryBuilder(point1, true).splineTo(point6OneRing, points6OneRingHeading).build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()).splineTo(point2, point2HeadingV2).build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()).lineTo(point3).build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()).lineTo(point4,
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end()).lineTo(point5, new MinVelocityConstraint(
//                        Arrays.asList(
//                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
//                        )
//                ),
//                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();
        Trajectory traj6 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-180))), true).lineToConstantHeading(point8).build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0, 0, Math.toRadians(-180))), true).splineTo(point9OneRing, point9OneRingHeading).build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end()).forward(30).build();

        //.splineTo(point9, point9Heading).build();
        //Trajectory traj9 = drive.trajectoryBuilder(traj8.end()).splineTo(point10, point10Heading).build();

        //Trajectory traj7 = drive.trajectoryBuilder(traj6.end()).splineTo(point9, point9Heading).build();


        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        //TODO create static class to store the position
        setTurretEncoderInitialEncoderPosition();
        turretEncoder.setInitialAngle(0);


        differentialMotor1  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        differentialMotor2 = hardwareMap.get(DcMotorEx.class, "right_differential_drive");
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
        servoIndexer = hardwareMap.get(ServoImplEx.class, "servo_indexer");
        shooterAngler = hardwareMap.get(ServoImplEx.class, "angle_servo");
        wobbleServo = hardwareMap.get(ServoImplEx.class, "wobble_arm");
        wobbleClawServo = hardwareMap.get(ServoImplEx.class, "wobble_claw");
        servoStick = hardwareMap.get(ServoImplEx.class, "stick_servo");


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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();

        popperTimer = new CountDownTimer();

        turret.defaultStateReset();
        intake.defaultStateReset();
        servoIntake.defaultStateReset();
        raisingServo.defaultStateReset();
        shooter.defaultStateReset();
        shooterIndexServo.defaultStateReset();
        shooterAngleServo.defaultStateRest();
        StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
        shooterIndexServo.servoOut();
        wobbleGoal.defaultStateReset();
        wobbleClaw.defaultStateReset();
        StateClass.setShootingTarget(StateClass.ShootingTarget.LEFT_POWERSHOT);
        StateClass.setShooterState(StateClass.ShooterState.STOPPED);
        StateClass.setTurretPositionState(StateClass.TurretPositionState.OFFTARGET);
        stickServo.defaultStateReset();


        CountDownTimer timer = new CountDownTimer();

        wobbleServo.setPosition(wobbleGoal.getServoPosition());
        StateClass.setWobbleArmState(StateClass.WobbleArmState.UP);

        wobbleClaw.servoClamped();
        wobbleClawServo.setPosition(wobbleClaw.getServoPosition());

        waitForStart();
        //wobbleGoal.servoDown();



        drive.setPoseEstimate(point1);
        if (isStopRequested()) return;

        // set current state to 1st step
        currentState = State.WAITING_FOR_WOBBLE;

        // go from starting position to shooting position
        drive.followTrajectoryAsync(traj1);

        // TODO: check config of starting stack & set it

        CountDownTimer wobbleGoalIteratingTimer = new CountDownTimer();
        wobbleGoalIteratingTimer.setTime(40);
        int wobbleGoalCounter = 1;
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            switch (currentState) {
                case WAITING_FOR_WOBBLE:
                    StateClass.setShootingTarget(StateClass.ShootingTarget.LEFT_POWERSHOT);
                    currentState = State.TRAJECTORY_1;
                    shooterIndexDrop.setTime(0);
                    ringCount = 4;
                    shots = 0;
                    shooter.setShooterSpeed(shot1Speed);
                    turret.setTurretFastMode();
                    wobbleGoal.servoDown();
                    break;

                case TRAJECTORY_1:
                    telemetry.addData("State", "Traj 1");

                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_1_DONE;
                    }
                    break;
                case TRAJECTORY_1_DONE:
                    telemetry.addData("State", "Traj 1 Done");

                    //shoot
                    wobbleGoal.servoDown();

                    if (StateClass.getWobbleArmState()== StateClass.WobbleArmState.DOWN) {
                        wobbleClaw.servoOpen();

                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(traj2);

                        turret.startTurret();
                        StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    }




//                    if (StateClass.getWobbleClawState()== StateClass.WobbleClawState.OPEN && StateClass.getWobbleArmState()== StateClass.WobbleArmState.DOWN) {
//                        currentState = State.TRAJECTORY_2;
//                        drive.followTrajectoryAsync(traj2);
//                    }

//                    telemetry.addData("State", "Traj 1 Done");
//                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);
//                    if (StateClass.getShootingSequenceState()==StateClass.ShootingSequence.NOT_SHOOTING) {
//                        currentState = State.TRAJECTORY_2;
//                        drive.followTrajectoryAsync(traj2);
//                        servoIntake.servoDown();
//                        stickServo.servoUp();
//                    }

                    break;

                case TRAJECTORY_2:
                    telemetry.addData("State", "Traj 2");
                    turret.startTurret();

                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_2_DONE;
                        leftPowerShotTimer.setTime(300);
                    }
                    break;
                case TRAJECTORY_2_DONE:
                    turret.startTurret();
                    telemetry.addData("State", "Traj 2 Done");
                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);

                    if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.NOT_SHOOTING) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(traj3);
                        servoIntake.servoDown();
                        stickServo.servoUp();
                    }

                    //shoot

                    //if shooting done, state = Trajectory 3
                    //drive.followTrajectoryAsync(traj3);

//                    currentState = State.TRAJECTORY_3;
//                    drive.followTrajectoryAsync(traj3);
//                    timer.setTime(3000);
//                    fullRev = false;
//                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    break;

                case TRAJECTORY_3:
                    telemetry.addData("State", "Traj 3");

                    //intake.intakeIn();
                    if (!drive.isBusy() ) {//&& timer.timeElapsed()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_3_DONE;
//                        shooterController.reset();
//                        ringCount = 4;
//                        shots = 0;
//                        StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
//                        StateClass.setShootingTarget(StateClass.ShootingTarget.HIGH_GOAL);
                    }
                    break;
                case TRAJECTORY_3_DONE:
                    telemetry.addData("State", "Traj 3 Done");

//                    fullRev = true;
//                    intake.intakeStop();
//
//                    shooterIndexDrop.setTime(0);
//
//                    shooter.setShooterSpeed(shot1Speed);
//                    turret.setTurretFastMode();
//                    turret.startTurret();
//                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);
                    //shoot

//                    if (StateClass.getShootingSequenceState() == StateClass.ShootingSequence.NOT_SHOOTING) {
                    intake.intakeIn();
                    currentState = State.TRAJECTORY_4;
                    drive.followTrajectoryAsync(traj4);
                    timer.setTime(2000);
//                    }
//                    break;

                case TRAJECTORY_4:

//                    telemetry.addData("State", "Traj 4");
//
//                    intake.intakeIn();
                    if (!drive.isBusy() && timer.timeElapsed()) {
                        fullRev = false;
                        StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                        // go to target zone
                        currentState = State.TRAJECTORY_4_DONE;
                        shooterController.reset();
                        ringCount = 4;
                        shots = 0;

                    }
                    break;
                case TRAJECTORY_4_DONE:
                    telemetry.addData("State", "Traj 4 Done");

                    //shoot
                    fullRev = true;
                    intake.intakeStop();
                    shooterIndexDrop.setTime(0);
                    shooter.setShooterSpeed(shot1Speed);
                    turret.setTurretFastMode();
                    turret.startTurret();
                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);

                    if (StateClass.getShootingSequenceState()== StateClass.ShootingSequence.NOT_SHOOTING) {
                        //drive.followTrajectoryAsync(traj5);
                        //timer.setTime(2000);
                        //intake.intakeIn();
                        //needsToShoot = false;
                        currentState = State.TURN_ONE;

                        needsToShoot = false;
                        drive.turnAsync(Math.toRadians(180));
                    }

                    //if shooting done, state = Trajectory 5
                    //drive.followTrajectoryAsync(traj5);
                    break;

                case TRAJECTORY_5:
                    telemetry.addData("State", "Traj 5");
                    if (!drive.isBusy()&&timer.timeElapsed()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_5_DONE;
                        shots = 0;
                        ringCount = 4;
                        StateClass.setShootingSequenceState(StateClass.ShootingSequence.REVING_UP);
                    }
                    break;
                case TRAJECTORY_5_DONE:
                    telemetry.addData("State", "Traj 5 Done");
                    shooterAngleServo.setServoPosition(.59);
                    //shoot
                    fullRev = true;
                    intake.intakeStop();
                    shooterIndexDrop.setTime(0);
                    shots = 0;
                    shooter.setShooterSpeed(shot1Speed);
                    turret.setTurretFastMode();
                    turret.startTurret();
                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_READY);

                    if (StateClass.getShootingSequenceState()== StateClass.ShootingSequence.NOT_SHOOTING) {
                        //currentState = State.TRAJECTORY_5;
                        //drive.followTrajectoryAsync(traj5);
                        //timer.setTime(3000);
                        //intake.intakeIn();
                        currentState = State.TURN_ONE;

                        needsToShoot = false;
                        drive.turnAsync(Math.toRadians(180));

                    }

//                    //shoot
//                    wobbleGoal.servoDown();
//                    wobbleClaw.servoOpen();
//                    if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.DOWN) {
//                        telemetry.addData("Wobble Arm", "Down");
//                    }
//
//                    if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.OPEN) {
//                        telemetry.addData("Wobble Claw", "Open");
//                    }
//
//                    if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.DOWN && StateClass.getWobbleClawState() == StateClass.WobbleClawState.OPEN) {
//                        drive.turnAsync(Math.toRadians(-180.0));
//                        currentState = State.TURN_ONE;
//                    }
//                    //if shooting done, state = Trajectory 6
//                    //drive.followTrajectoryAsync(traj6);
                    break;

                case TURN_ONE:
                    telemetry.addData("State", "Turn 1");
                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectoryAsync(traj6);
                    }
                    break;

                case TRAJECTORY_6:
                    telemetry.addData("State", "Traj 6");

                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_6_DONE;
                    }
                    break;
                case TRAJECTORY_6_DONE:
                    telemetry.addData("State", "Traj  6 done");

                    wobbleClaw.servoClamped();

                    if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.CLAMPED) {
                        drive.turnAsync(Math.toRadians(-180.0), 20, 2);

                        currentState = State.TURN_TWO;
//                        wobbleGoal.servoUp();
//                        if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.UP) {
//                            currentState = State.TURN_TWO;
//                        }
                    }
                    break;
                case TURN_TWO:
                    telemetry.addData("State", "Turn 2");

                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_7;
                        drive.followTrajectoryAsync(traj7);

                    }
                    break;

                case TRAJECTORY_7:
                    telemetry.addData("State", "Traj 7");


                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_7_DONE;
                    }
                    break;
                case TRAJECTORY_7_DONE:
                    telemetry.addData("State", "Traj 7 Done");

                    wobbleGoal.servoDown();
                    wobbleClaw.servoOpen();

                    if (StateClass.getWobbleArmState() == StateClass.WobbleArmState.DOWN && StateClass.getWobbleClawState() == StateClass.WobbleClawState.OPEN) {
                        wobbleClaw.servoBack();
                        if (StateClass.getWobbleClawState() == StateClass.WobbleClawState.BACK) {
                            wobbleGoal.servoBack();
                        }
                        stickServo.servoDown();
                        currentState = State.TRAJECTORY_8;
                        drive.followTrajectoryAsync(traj8);

                    }
                    break;
//
                case TRAJECTORY_8:
                    if (!drive.isBusy()) {
                        // go to target zone
                        currentState = State.TRAJECTORY_8_DONE;
                    }
                    break;
//                case TRAJECTORY_8_DONE:
//                    //shoot
//
//                    //if shooting done, state = Trajectory 9
//                    //drive.followTrajectoryAsync(traj9);
//                    break;


            }
            drive.update();
            myPose = drive.getPoseEstimate();
            PoseStorage.turretAngle = turretEncoder.getTurretAngle();
            PoseStorage.currentPose = myPose;
            updateMechanisms();
            updateTargetPosition();
            reportServoTelemetry();
            reportTurretTelemetry();
            reportIntakeTelemetry();
            reportShooterTelemetry();





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

    }
    public CountDownTimer leftPowerShotTimer = new CountDownTimer();
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
                    telemetry.addData("Ring Count", ringCount);
                    if (StateClass.getServoRaiserState() == StateClass.ServoRaiserState.UP) {
                        telemetry.addData("Up", "up");

                        if (StateClass.getShooterState() == StateClass.ShooterState.ATSPEED) {
                            readied = true;

                            if (StateClass.getIndexReady() == StateClass.IndexReady.INDEX_READY) {
                                if (StateClass.getTurretPositionState() == StateClass.TurretPositionState.ONTARGET) {
                                    if (currentState == State.TRAJECTORY_2_DONE && leftPowerShotTimer.timeElapsed()) {
                                        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT) {
                                            shooterIndexServo.servoIn();
                                            ringCount -= 1;
                                            shots += 1;

                                        }
                                        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN) {
                                            shooterIndexServo.servoOut();
                                            if (shots == 1) {
                                                shooter.setShooterSpeed(shot2Speed);
                                                if (currentState == State.TRAJECTORY_2_DONE) {
                                                    StateClass.setShootingTarget(StateClass.ShootingTarget.MIDDLE_POWERSHOT);
                                                }

                                            }
                                            if (shots == 2) {
                                                shooter.setShooterSpeed(shot3Speed);
                                                if (currentState == State.TRAJECTORY_2_DONE) {
                                                    StateClass.setShootingTarget(StateClass.ShootingTarget.RIGHT_POWERSHOT);
                                                }

                                            }
                                        }
                                    }
                                    else {
                                        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.OUT) {
                                            shooterIndexServo.servoIn();
                                            ringCount -= 1;
                                            shots += 1;

                                        }
                                        if (StateClass.getShooterServoState() == StateClass.ShooterServoState.IN) {
                                            shooterIndexServo.servoOut();
                                            if (shots == 1) {
                                                shooter.setShooterSpeed(shot2Speed);
                                                if (currentState == State.TRAJECTORY_2_DONE) {
                                                    StateClass.setShootingTarget(StateClass.ShootingTarget.MIDDLE_POWERSHOT);
                                                }

                                            }
                                            if (shots == 2) {
                                                shooter.setShooterSpeed(shot3Speed);
                                                if (currentState == State.TRAJECTORY_2_DONE) {
                                                    StateClass.setShootingTarget(StateClass.ShootingTarget.RIGHT_POWERSHOT);
                                                }

                                            }
                                        }
                                    }

                                }

                            }

                        }

                    } else {
                        telemetry.addData("Ready to shoot", "Servo Raiser is not up");
                    }

                } else {
                    shooterIndexServo.servoOut();
                    StateClass.setShootingSequenceState(StateClass.ShootingSequence.NOT_SHOOTING);
                    readied = false;
                    StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
                    StateClass.setShootingTarget(StateClass.ShootingTarget.HIGH_GOAL);

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
            raisingServo.servoDown();
            StateClass.setShooterState(StateClass.ShooterState.STOPPED);
            shooterIndexServo.servoOut();
            readied = false;
            StateClass.setIndexReady(StateClass.IndexReady.INDEX_NOTREADY);
            turret.stopTurret();
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
        servoRaiser.setPosition(raisingServo.getServoPosition());

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

        if (needsToShoot) { //(StateClass.getShooterState() != StateClass.ShooterState.STOPPED) {
            shooterController.setTargetPosition(-1);
            telemetry.addData("measured position", -shooterMotor1.getVelocity()/shooter.getShooterSpeed());

            shooterPIDPower = shooterController.update(-shooterMotor1.getVelocity()/shooter.getShooterSpeed());
            telemetry.addData("PID", "updating pid");
            //shooterPIDPower = 0.5;
            shooterMotor1.setPower(shooterPIDPower);
            shooterMotor2.setPower(shooterPIDPower);
            setToZero = false;

            //shooterMotor1.setVelocity(shooter.getShooterSpeed());
            //shooterMotor2.setPower(shooter.getShooterSpeed()/shooterMotor2.getMotorType().getAchieveableMaxTicksPerSecond());
        }
        else  {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
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

            if (currentState == State.TRAJECTORY_1 || currentState ==State.TRAJECTORY_2 || currentState ==State.TRAJECTORY_1_DONE) {
                targetAngle = -180;
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

    public void calculateShooterInfo(double distance) {
        //replace with some function that does something
        //shooter.setShooterSpeed(.8);
    }

    public void updateTargetPosition() {
        switch (StateClass.getShootingTarget()) {
            case HIGH_GOAL:
                targetX = 72;
                targetY = 34.125;
                if (shooterAngleServo.servoPosition != .59) {
                    shooterAngleServo.setServoPosition(.58);
                }
                break;
            case LEFT_POWERSHOT:
                targetX = 72;
                targetY = (22.75-6.25);;//(22.75-4.25);
                shooterAngleServo.setServoPosition(.55);
                break;
            case MIDDLE_POWERSHOT:
                targetX = 72;
                targetY = (22.75-14);//11.0);
                shooterAngleServo.setServoPosition(.55);
                break;
            case RIGHT_POWERSHOT:
                targetX = 72;
                targetY = (22.75-21.5);
                shooterAngleServo.setServoPosition(.55);
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
        switch (StateClass.getTurretPositionState()) {
            case OFFTARGET:
                telemetry.addData("Target", "On Target");
                break;
            case ONTARGET:
                telemetry.addData("Target", "Off Target");
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

        switch (StateClass.getShootingSequenceState()) {
            case NOT_SHOOTING:
                telemetry.addData("Sequence", "NOT SHOOTING");
                break;
            case REVING_UP:
                telemetry.addData("Sequence", "REeving Up");
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

    enum State {
        WAITING_FOR_WOBBLE,
        TRAJECTORY_1,
        TRAJECTORY_1_DONE,
        TRAJECTORY_2,
        TRAJECTORY_2_DONE,
        TRAJECTORY_3,
        TRAJECTORY_3_DONE,
        TRAJECTORY_4,
        TRAJECTORY_4_DONE,
        TRAJECTORY_5,
        TRAJECTORY_5_DONE,
        TRAJECTORY_6,
        TRAJECTORY_6_DONE,
        TRAJECTORY_7,
        TRAJECTORY_7_DONE,
        TRAJECTORY_8,
        TRAJECTORY_8_DONE,
        TRAJECTORY_9,
        TRAJECTORY_9_DONE,
        TURN_ONE,
        DONE_TURNING_ONE,
        TURN_TWO,
        DONE_TURNING_TWO,
        INIT
    }
}