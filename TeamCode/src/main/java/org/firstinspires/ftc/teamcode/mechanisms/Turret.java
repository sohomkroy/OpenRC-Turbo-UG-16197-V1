package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;

@Config
    public class Turret {
        public static double kP = .1;
        public static double kD = 0.002;
        public static double kS = 0;
        public static double kI = .00015;
        public static double kV = 0;
        public static double kA = 0;
        //0.005
        CountDownTimer targetTimer = new CountDownTimer();
        private Differential differential;
        private TurretEncoder turretEncoder;
        private double thresholdTime = 20;
        private double turretTargetPosition;
        //0.1
        private double turretSlowSpeed = 0.5;
        private PIDCoefficients turretPIDCoefficients = new PIDCoefficients(kP, kI, kD);
        public PIDFController controller = new PIDFController(turretPIDCoefficients, kV, kA, 0);
    private double turretThreshold = .3;
    private double turretFastSpeed = .5;

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

        public void setThresholdTime(double thresholdTime) {
            this.thresholdTime = thresholdTime;
        }

    public double getControllerOutput() {
        return controllerOutput;
    }
    private boolean onTarget;

    public void updateTurret() {
        switch (StateClass.getTurretMovement()) {
            case MOVING:
                controllerOutput = controller.update(turretEncoder.getTurretAngle());
                if (controllerOutput>0) {
                    if (Math.abs(controller.getLastError()) > turretThreshold*.75) {
                        controllerOutput+=kS;
                    }
                }
                else if (controllerOutput<0) {
                    if (Math.abs(controller.getLastError()) > turretThreshold*.75) {
                        controllerOutput-=kS;
                    }
                }
                //controllerOutput = kS;
                differential.setTurretSpeed(controllerOutput);
                break;
            case STOPPED:
                controllerOutput = controller.update(turretEncoder.getTurretAngle());
                if (controllerOutput>0) {
                    if (Math.abs(controller.getLastError()) > turretThreshold*.75) {
                        controllerOutput+=kS;
                    }
                }
                else if (controllerOutput<0) {
                    if (Math.abs(controller.getLastError()) > turretThreshold*.75) {
                        controllerOutput-=kS;
                    }
                }
                //controllerOutput = kS;
                differential.setTurretSpeed(controllerOutput);
                break;
//                differential.setTurretSpeed(0);
//                break;
        }
        onTargetCheck();
    }

    public boolean onTarget() {
        return onTarget;
    }

    private void onTargetCheck() {
        if (StateClass.getTurretMovement() == StateClass.TurretMovement.STOPPED) {
            onTarget =  false;
        } else {
            if (StateClass.getShootingTarget() == StateClass.ShootingTarget.HIGH_GOAL) {
                turretThreshold = .5;
            }
            else {
                turretThreshold = .35;
            }
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
