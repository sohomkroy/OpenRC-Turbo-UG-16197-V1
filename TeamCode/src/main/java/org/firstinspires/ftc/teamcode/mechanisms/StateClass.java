package org.firstinspires.ftc.teamcode.mechanisms;

public class StateClass {
    public enum IntakeState {
        IN,
        OUT,
        STOPPED
    }

    private IntakeState intakeState;
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public enum ServoIntakeState {
        UP,
        BACK,
        DOWN,
        MOVING
    }

    private ServoIntakeState servoIntakeState;
    public void setServoIntakeState(ServoIntakeState servoIntakeState) {
        this.servoIntakeState = servoIntakeState;
    }

    public ServoIntakeState getServoIntakeState() {
        return servoIntakeState;
    }

    public enum TurretMovementSpeed {
        HIGHPOWER,
        LOWPOWER
    }

    private TurretMovementSpeed turretMovementSpeed;
    public void setTurretMovementSpeed(TurretMovementSpeed turretMovementSpeed) {
        this.turretMovementSpeed = turretMovementSpeed;
    }

    public TurretMovementSpeed getTurretMovementSpeed() {
        return this.turretMovementSpeed;
    }

    public enum TurretMovement {
        MOVING,
        STOPPED
    }

    private TurretMovement turretMovement;

    public void setTurretMovement(TurretMovement turretMovement) {
        this.turretMovement = turretMovement;
    }

    public TurretMovement getTurretMovement() {
        return turretMovement;
    }

    public enum TurretPositionState {
        ONTARGET,
        OFFTARGET
    }

    private TurretPositionState turretPositionState;

    public void setTurretPositionState(TurretPositionState turretPositionState) {
        this.turretPositionState = turretPositionState;
    }

    public TurretPositionState getTurretPositionState() {
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
