package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 2.28346/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.195497; // in; distance between the left and right wheels


    //14.1938583
    //14.063
    //14.0952
    //14.073!!!!! this is guuds
    //14.153
    //14.16159333
    //14.17459333
    //14.16489333
    //14.21485436

    //14.240466728514
    //14.2650466728514
    //14.125987~

    //14.225987
    //14.229287
    //14.228387

    //14.195587

    public static double FORWARD_OFFSET = -6.748543307; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.9988069;
    //0.997
    // 0.978
    //1.0018042
    //0.99832557
    //0.99731065

    //0.99423849
    //0.99463849

    //0.99291979583
    //0.99098357608

    //0.9988069
    public static double Y_MULTIPLIER = 0.99973090283;
    //0.972
    //0.979
    //0.980
    //0.99055874
    //0.99123747463

    //0.989035256

    //0.99973090283

    //pid shit
    //max velo: 130 --> 90% = 117
    //kF = 0.500516, 0.4871, 0.495 --> 0.49536

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_rear_drive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        frontEncoder = new   Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        // TODO: reverse any encoders using Encoder.setDirection
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()*X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition()*X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCurrentPosition()*Y_MULTIPLIER)
        );
    }



    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()*X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()*X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()*Y_MULTIPLIER)
        );
    }
}
