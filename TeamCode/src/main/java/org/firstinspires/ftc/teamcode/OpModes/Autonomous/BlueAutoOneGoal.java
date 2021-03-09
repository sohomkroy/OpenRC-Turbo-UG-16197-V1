package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoConstants.*;

@Autonomous
public class BlueAutoOneGoal extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    enum State {
        START_TO_SHOOT,
        TURN_1,
        SHOOT_TO_STACK,
        TURN_2,
        STACK_TO_SHOOT,
        SHOOT_TO_TARGET,
        TARGET_TO_PARK,
        IDLE
    }

    State currentState = State.IDLE;

    // main trajectories
    Trajectory startToShoot = drive.trajectoryBuilder(startPos).splineTo(shootPos, shootHeading).build();
    Trajectory shootToStack = drive.trajectoryBuilder(new Pose2d(shootPos, shootHeading)).lineTo(stackPos).build();
    Trajectory stackToShoot = drive.trajectoryBuilder(new Pose2d(stackPos, stackHeading)).lineTo(shootPos2).build();
    Trajectory shootToTarget = drive.trajectoryBuilder(new Pose2d(shootPos, shootHeading)).splineTo(targetPos, targetHeading).build();
    Trajectory targetToPark = drive.trajectoryBuilder(new Pose2d(targetPos, targetHeading)).lineTo(parkPos).build();

    int stackSize = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;

        // set current state to 1st step
        currentState = State.START_TO_SHOOT;

        // go from starting position to shooting position
        drive.followTrajectoryAsync(startToShoot);

        // TODO: check config of starting stack & set it

        if(stackSize == 0){
            // overall order: startToShoot, shootToTarget, targetToPark
            /*
            TODO: Add mechanism code following this order (might have to modify switch statement/enums):
            MECH: shoot preloaded disks (rotate turret as needed)
            TRAJ: shootToTarget
            MECH: deposit wobble goal
            TRAJ: targetToPark
            END: park
             */
            while (opModeIsActive() && !isStopRequested()) {
                switch(currentState) {
                    case START_TO_SHOOT:
                        if(!drive.isBusy()){
                            // go to target zone
                            currentState = State.SHOOT_TO_TARGET;
                            drive.followTrajectoryAsync(shootToTarget);
                        }
                        break;
                    case SHOOT_TO_TARGET:
                        if(!drive.isBusy()){
                            // go to line and park
                            currentState = State.TARGET_TO_PARK;
                            drive.followTrajectoryAsync(targetToPark);
                        }
                        break;
                    case TARGET_TO_PARK:
                        if(!drive.isBusy()){
                            // do nothing in idle
                            currentState = State.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }
                drive.update();
                PoseStorage.currentPose = drive.getPoseEstimate();
            }
        }else {
            // overall order: startToShoot, shootToStack, stackToShoot, shootToTarget, targetToPark
            /*
            TODO: Add mechanism code following this order (might have to modify switch statement/enums):
            MECH: shoot preloaded disks
            TRAJ: turn 120º clockwise
            TRAJ: shootToStack
            MECH: intake start stack disk(s)
            TRAJ: turn 120º counter-clockwise
            TRAJ: stackToShoot
            MECH: shoot disk(s)
            TRAJ: shootToTarget
            MECH: deposit wobble goal
            TRAJ: targetToPark
            END: park
             */
            while (opModeIsActive() && !isStopRequested()) {
                switch (currentState) {
                    case START_TO_SHOOT:
                        if(!drive.isBusy()){
                            // turn 120º clockwise
                            currentState = State.TURN_1;
                            drive.turnAsync(stackHeading);
                        }
                        break;
                    case TURN_1:
                        if(!drive.isBusy()){
                            // go to stack
                            currentState = State.SHOOT_TO_STACK;
                            drive.followTrajectoryAsync(shootToStack);
                        }
                        break;
                    case SHOOT_TO_STACK:
                        if(!drive.isBusy()){
                            // turn 120º counter-clockwise
                            currentState = State.TURN_2;
                            drive.turnAsync(shootHeading - stackHeading);
                        }
                        break;
                    case TURN_2:
                        if(!drive.isBusy()){
                            // go to second shooting position
                            currentState = State.STACK_TO_SHOOT;
                            drive.followTrajectoryAsync(stackToShoot);
                        }
                        break;
                    case STACK_TO_SHOOT:
                        if(!drive.isBusy()){
                            // go to target zone
                            currentState = State.SHOOT_TO_TARGET;
                            drive.followTrajectoryAsync(shootToTarget);
                        }
                        break;
                    case SHOOT_TO_TARGET:
                        if(!drive.isBusy()){
                            // go to line and park
                            currentState = State.TARGET_TO_PARK;
                            drive.followTrajectoryAsync(targetToPark);
                        }
                        break;
                    case TARGET_TO_PARK:
                        if(!drive.isBusy()){
                            // do nothing in idle
                            currentState = State.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }
                drive.update();
                PoseStorage.currentPose = drive.getPoseEstimate();
            }
        }
    }
}
