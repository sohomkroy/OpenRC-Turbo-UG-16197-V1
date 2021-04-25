package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class TiltingRotationServo {
    public static double frontDegreePoint = .80;//0
    public static double backDegreePoint = .04;//180

    public static double angleToPosition(double angle) {
        return Range.clip(frontDegreePoint + (frontDegreePoint-backDegreePoint)/180*(angle), 0, 1);

        //return (backDegreePoint-frontDegreePoint)/180*angle + (3*frontDegreePoint-backDegreePoint)/2;
    }
}
