package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

public class Turret {
    private StateClass stateClass;
    private Differential differential;
    private TurretEncoder turretEncoder;

    private double turretSlowSpeed = .1;
    private double turretFastSpeed = .2;

    private double turretTargetPosition;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private PIDCoefficients turretPIDCoefficients = new PIDCoefficients(kP, kI, kD);
    private PIDFController controller = new PIDFController(turretPIDCoefficients);


    public Turret(StateClass stateClass, Differential differential, TurretEncoder turretEncoder) {
        this.stateClass = stateClass;
        this.stateClass.setTurretMovement(StateClass.TurretMovement.STOPPED);
        this.differential = differential;
        this.turretEncoder = turretEncoder;
        controller.setOutputBounds(-turretSlowSpeed, turretSlowSpeed);
    }

    public void setTurretTargetPosition(double angle) {
        turretTargetPosition = angle;
        controller.setTargetPosition(angle);
    }

    public void setTurretSlowMode() {
        stateClass.setTurretMovementSpeed(StateClass.TurretMovementSpeed.LOWPOWER);
        controller.setOutputBounds(-turretSlowSpeed, turretSlowSpeed);
    }

    public void setTurretFastMode() {
        stateClass.setTurretMovementSpeed(StateClass.TurretMovementSpeed.HIGHPOWER);
        controller.setOutputBounds(-turretFastSpeed, turretFastSpeed);
    }

    public void startTurret() {
        this.stateClass.setTurretMovement(StateClass.TurretMovement.MOVING);
    }
    public void stopTurret() {
        this.stateClass.setTurretMovement(StateClass.TurretMovement.STOPPED);
    }

    public void updateTurret() {
        switch (stateClass.getTurretMovement()) {
            case MOVING:
                differential.setTurretSpeed(controller.update(turretEncoder.getTurretAngle()));
                break;
            case STOPPED:
                differential.setTurretSpeed(0);
                break;
        }
    }
}
