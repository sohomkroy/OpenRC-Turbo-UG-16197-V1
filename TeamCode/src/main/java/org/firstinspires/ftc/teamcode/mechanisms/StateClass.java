package org.firstinspires.ftc.teamcode.mechanisms;

public class StateClass {
    enum IntakeState {
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

    enum ServoIntakeState {
        UP,
        BACK,
        DOWN,
        MOVING
    }

    enum TurretMovementSpeed {
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

    enum TurretMovement {
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

    enum TurretPositionState {
        ONTARGET,
        OFFTARGET
    }

    enum ServoRaiserState {
        UP,
        DOWN,
        MOVING
    }
    enum ShooterServoState {
        IN,
        OUT,
        MOVING
    }
    enum ShooterState {
        ATSPEED,
        WINDINGUP,
        STOPPED
    }
    enum AngleAdjustState {
        ONTARGET,
        MOVING,
        STOPPED
    }

    public StateClass() {

    }
}
