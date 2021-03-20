package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

@Config
public class Turret {
    private Differential differential;
    private TurretEncoder turretEncoder;

    private double thresholdTime = 20;

    public void setThresholdTime(double thresholdTime) {
        this.thresholdTime = thresholdTime;
    }

    public static double kP = .08;
    public static double kD = 0.005;

    private double turretTargetPosition;
    public static double kS = 0;
    public static double kI = 0;
    private double turretSlowSpeed = 0;
    CountDownTimer targetTimer = new CountDownTimer();
    public static double kV = 0;
    public static double kA = 0;
    private PIDCoefficients turretPIDCoefficients = new PIDCoefficients(kP, kI, kD);
    private PIDFController controller = new PIDFController(turretPIDCoefficients, kV, kA, 0);

//    private PIDFController controller = new PIDFController(turretPIDCoefficients, 0, 0, kS, new Function2<Double, Double, Double>() {
//        @Override
//        public Double invoke(Double position, Double velocity) {
//            return -0.2;
//        }
//    });
    public void defaultStateReset(){
        StateClass.setTurretMovement(StateClass.TurretMovement.MOVING);
        StateClass.setTurretMovementSpeed(StateClass.TurretMovementSpeed.LOWPOWER);

        setTurretFastMode();
    }

    public Turret(Differential differential, TurretEncoder turretEncoder) {
        this.differential = differential;
        this.turretEncoder = turretEncoder;
        controller.setOutputBounds(-turretSlowSpeed, turretSlowSpeed);

    }

    public void setTurretTargetPosition(double angle) {
        //turretPIDCoefficients = new PIDCoefficients(kP, kI, kD);
        //controller = new PIDFController(turretPIDCoefficients, kV, kA, kS);
        this.turretTargetPosition = angle;
        controller.setTargetPosition(turretTargetPosition);

    }

    public void setTurretSlowMode() {
        StateClass.setTurretMovementSpeed(StateClass.TurretMovementSpeed.LOWPOWER);
        controller.setOutputBounds(-turretSlowSpeed, turretSlowSpeed);
    }

    public void setTurretFastMode() {
        StateClass.setTurretMovementSpeed(StateClass.TurretMovementSpeed.HIGHPOWER);
        controller.setOutputBounds(-turretFastSpeed, turretFastSpeed);
    }

    public void startTurret() {
        StateClass.setTurretMovement(StateClass.TurretMovement.MOVING);
    }
    public void stopTurret() {
        StateClass.setTurretMovement(StateClass.TurretMovement.STOPPED);
    }
    private double controllerOutput;
    public void updateTurret() {
        switch (StateClass.getTurretMovement()) {
            case MOVING:
                controllerOutput = controller.update(turretEncoder.getTurretAngle());
                if (controllerOutput>0) {
                    controllerOutput+=kS;
                }
                else if (controllerOutput<0) {
                    controllerOutput-=kS;
                }
                //controllerOutput = kS;
                differential.setTurretSpeed(controllerOutput);
                break;
            case STOPPED:
                differential.setTurretSpeed(0);
                break;
        }
        onTargetCheck();
    }

    private double turretThreshold = 1;
    private boolean onTarget;
    private double turretFastSpeed = .6;

    public boolean onTarget() {
        return onTarget;
    }

    private void onTargetCheck() {
        if (StateClass.getTurretMovement() == StateClass.TurretMovement.STOPPED) {
            onTarget =  false;
        } else {
            onTarget = Math.abs(controller.getLastError()) < turretThreshold;
            if (onTarget) {
                StateClass.setTurretPositionState(StateClass.TurretPositionState.ONTARGET);

//                if (targetTimer.timeElapsed()) {
//                    StateClass.setTurretPositionState(StateClass.TurretPositionState.ONTARGET);
//                }
            }
            else {
                StateClass.setTurretPositionState(StateClass.TurretPositionState.OFFTARGET);

            }

        }
    }
}
