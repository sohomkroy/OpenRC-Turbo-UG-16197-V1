package org.firstinspires.ftc.teamcode.mechanisms;

public class StateClass {
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



    enum ServoRaiserState {
        UP,
        DOWN,
        MOVING
    }
    public enum ShooterServoState {
        IN,
        OUT,
        MOVING
    }
    public enum ShooterState {
        ATSPEED,
        WINDINGUP,
        STOPPED
    }
    public enum AngleAdjustState {
        ONTARGET,
        MOVING,
        STOPPED
    }

    public StateClass() {

    }
}
