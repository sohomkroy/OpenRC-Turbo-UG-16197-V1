/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of org.firstinspires.ftc.teamcode.OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear org.firstinspires.ftc.teamcode.OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicTestMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_differential_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_differential_drive");

        // Most robots need the motor on one side to be reversed to org.firstinspires.ftc.teamcode.drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.getMotorType().setAchieveableMaxRPMFraction(1.0);
        rightDrive.getMotorType().setAchieveableMaxRPMFraction(1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each org.firstinspires.ftc.teamcode.drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to org.firstinspires.ftc.teamcode.drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to org.firstinspires.ftc.teamcode.drive straight.
            double left_power = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to org.firstinspires.ftc.teamcode.drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(left_power);
            rightDrive.setPower(left_power);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Power", left_power);
            telemetry.addData("Velo1", leftDrive.getVelocity());
            telemetry.addData("Velo2", rightDrive.getVelocity());
            telemetry.addData("Pos1", leftDrive.getCurrentPosition());
            telemetry.addData("Pos2", rightDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}