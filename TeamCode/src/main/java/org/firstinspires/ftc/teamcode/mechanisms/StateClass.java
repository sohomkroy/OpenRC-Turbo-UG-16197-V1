package org.firstinspires.ftc.teamcode.mechanisms;

public class StateClass {

    private static ShootingTarget shootingTarget;

    public static ShootingTarget getShootingTarget() {
        return shootingTarget;
    }

    public static void setShootingTarget(ShootingTarget shootingTarget) {
        StateClass.shootingTarget = shootingTarget;
    }

    public enum ShootingTarget {
        HIGH_GOAL,
        LEFT_POWERSHOT,
        MIDDLE_POWERSHOT,
        RIGHT_POWERSHOT
    }




    private static IndexReady indexReady;
    private static ShootingSequence shootingSequenceState;

    public static IndexReady getIndexReady() {
        return indexReady;
    }

    public static void setIndexReady(IndexReady indexReady) {
        StateClass.indexReady = indexReady;
    }

    public static ShootingSequence getShootingSequenceState() {
        return shootingSequenceState;
    }

    public static void setShootingSequenceState(ShootingSequence shootingSequenceState) {
        StateClass.shootingSequenceState = shootingSequenceState;
    }

    public enum IndexReady {
        INDEX_READY,
        INDEX_NOTREADY
    }

    public enum ShootingSequence {
        REVING_UP,
        NOT_SHOOTING,
    }



    public enum IntakeState {
        IN,
        OUT,
        STOPPED
    }

    private static IntakeState intakeState;
    public static void setIntakeState(IntakeState intakeState) {
        StateClass.intakeState = intakeState;
    }

    public static IntakeState getIntakeState() {
        return intakeState;
    }

    public enum ServoIntakeState {
        UP,
        BACK,
        DOWN,
        MOVING_UP,
        MOVING_DOWN,
        MOVING_BACK,
    }

    private static ServoIntakeState servoIntakeState;
    public static void setServoIntakeState(ServoIntakeState servoIntakeState) {
        StateClass.servoIntakeState = servoIntakeState;
    }
    public static ServoIntakeState getServoIntakeState() {
        return servoIntakeState;
    }

    public enum WobbleArmState {
        UP,
        BACK,
        DOWN,
        MOVING_UP,
        MOVING_DOWN,
        MOVING_BACK,
    }

    private static WobbleArmState wobbleArmState;
    public static void setWobbleArmState(WobbleArmState wobbleArmState) {
        StateClass.wobbleArmState = wobbleArmState;
    }

    public static WobbleArmState getWobbleArmState() {
        return wobbleArmState;
    }

    public enum TurretMovementSpeed {
        HIGHPOWER,
        LOWPOWER
    }

    private static TurretMovementSpeed turretMovementSpeed;
    public static void setTurretMovementSpeed(TurretMovementSpeed turretMovementSpeed) {
        StateClass.turretMovementSpeed = turretMovementSpeed;
    }

    public static TurretMovementSpeed getTurretMovementSpeed() {
        return StateClass.turretMovementSpeed;
    }

    public enum TurretMovement {
        MOVING,
        STOPPED
    }

    private static TurretMovement turretMovement;

    public static void setTurretMovement(TurretMovement turretMovement) {
        StateClass.turretMovement = turretMovement;
    }

    public static TurretMovement getTurretMovement() {
        return turretMovement;
    }

    public enum TurretPositionState {
        ONTARGET,
        OFFTARGET
    }

    private static TurretPositionState turretPositionState;

    public static void setTurretPositionState(TurretPositionState turretPositionState) {
        StateClass.turretPositionState = turretPositionState;
    }

    public static TurretPositionState getTurretPositionState() {
        return turretPositionState;
    }



    public enum ServoRaiserState {
        UP,
        DOWN,
        MOVING_UP,
        MOVING_DOWN
    }
    private static ServoRaiserState servoRaiserState;

    public static void setServoRaiserState(ServoRaiserState servoRaiserState) {
        StateClass.servoRaiserState = servoRaiserState;
    }

    public static ServoRaiserState getServoRaiserState() {
        return servoRaiserState;
    }

    public enum ShooterServoState {
        IN,
        OUT,
        MOVING_IN,
        MOVING_OUT
    }

    private static ShooterServoState shooterServoState;

    public static void setShooterServoState(ShooterServoState shooterServoState) {
        StateClass.shooterServoState = shooterServoState;
    }

    public static ShooterServoState getShooterServoState() {
        return shooterServoState;
    }

    public enum ShooterState {
        ATSPEED,
        WINDINGUP,
        STOPPED
    }

    private static ShooterState shooterState;

    public static ShooterState getShooterState() {
        return shooterState;
    }

    public static void setShooterState(ShooterState shooterState) {
        StateClass.shooterState = shooterState;
    }

    public enum WobbleClawState {
        CLAMPED,
        OPEN,
        BACK,
        MOVING_CLAMPED,
        MOVING_OPEN,
        MOVING_BACK
    }

    private static WobbleClawState wobbleClawState;

    public static void setWobbleClawState(WobbleClawState wobbleClawState) {
        StateClass.wobbleClawState = wobbleClawState;
    }

    public static WobbleClawState getWobbleClawState() {
        return wobbleClawState;
    }



    public StateClass() {

    }
}
