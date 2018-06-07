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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import java.lang.Math;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RecovererDrive", group="Recoverer")
public class RecovererDriveTest extends OpMode{

    /* Declare OpMode members. */
    HardwareRecoverer robot       = new HardwareRecoverer(); // use the class created to define a Pushbot's hardware

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    boolean armStateOpen = true;
    boolean hasExecuted = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

    //}

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    //@Override
    //public void init_loop() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double correction = 0.0;
    boolean isntStrafing = true;
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;
    static final double     desiredMotorRevs        = 3;
    static final double     totalRelicDistance      = COUNTS_PER_MOTOR_REV * desiredMotorRevs;
    boolean armStateOpen2 = false;
    boolean armStateOpen3 = false;
    boolean hasExecuted2 = true;
    boolean hasExecuted3 = true;


    @Override
    public void loop() {


        double right_side;
        double left_side;
        double turn;
        boolean slow;
        double slowDub;
        double strafeHeading;
        double relicMove;
        double right;
        double currentRelic = robot.relicextend.getCurrentPosition();
        String debug = "nothing";

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_side = -gamepad1.left_stick_y;
        right_side = gamepad1.right_stick_y;
        turn = gamepad1.right_stick_x;
        slow = gamepad1.right_bumper;
        relicMove = -gamepad2.right_stick_y;


        slowDub = (slow) ? 6:2;

        slowDub = slowDub - gamepad1.right_trigger;

        if (Math.abs(turn) > 0.5)
        {
            if (isntStrafing)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                correction = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                isntStrafing = false;
                debug = "Updating Correction";
                if (correction < -90)
                {
                    correction = 360 + correction;
                }
            }
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            strafeHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("StrafeHeading", "%s", strafeHeading);
            if (strafeHeading < -90 && correction > 180)
            {
                strafeHeading = 360 + strafeHeading;
            }

            if (strafeHeading <= correction + 1.5 && strafeHeading >= correction - 1.5)
            {
                robot.ldBack.setPower(-turn/ slowDub);
                robot.ldFront.setPower(turn/ slowDub);
                robot.rdBack.setPower(-turn/ slowDub);
                robot.rdFront.setPower(turn/ slowDub);
                debug = "Working Fine";
            }
            else if (strafeHeading < correction - 1.5)
            {
                robot.ldBack.setPower(-turn / slowDub - .15 * (turn/Math.abs(turn)));
                robot.ldFront.setPower(turn / slowDub);
                robot.rdBack.setPower(-turn / slowDub - .15 * (turn/Math.abs(turn)));
                robot.rdFront.setPower(turn / slowDub);
                debug = "Back is slower";
            }
            else if (strafeHeading > correction + 1.5)
            {
                robot.ldBack.setPower(-turn / slowDub);
                robot.ldFront.setPower(turn / slowDub - .15 * (turn/Math.abs(turn)));
                robot.rdBack.setPower(-turn / slowDub);
                robot.rdFront.setPower(turn / slowDub - .15 * (turn/Math.abs(turn)));
                debug = "Front is slower";
            }
        }
        else
        {
            isntStrafing = true;
            robot.ldBack.setPower(left_side/slowDub);
            robot.ldFront.setPower(left_side/slowDub);
            robot.rdFront.setPower(right_side/slowDub);
            robot.rdBack.setPower(right_side/slowDub);
        }

        if (gamepad1.b)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    });
        }



        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad2.right_bumper && hasExecuted) {
            if (armStateOpen) {
                armStateOpen = Boolean.FALSE;
                clawOffset = -.5;

            }   else {
                armStateOpen = Boolean.TRUE;
                clawOffset = 0;
            }
            hasExecuted = false;
        }

        if (gamepad2.left_bumper && hasExecuted)
        {
            armStateOpen = Boolean.TRUE;
            clawOffset = -.20;
            hasExecuted = false;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (hasExecuted == false && gamepad2.right_bumper == false){
            // clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO - clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO + clawOffset);
            hasExecuted = true;
       }



        // Use gamepad buttons to move the arm up (Y) and down (A)

        if (gamepad2.y && robot.touchSensor.getState())
        {
            robot.arm.setPower(robot.ARM_UP_POWER);
        }
        else if (gamepad2.a)
        {
            robot.arm.setPower(-robot.ARM_UP_POWER);
        }
        else
        {
            robot.arm.setPower(0);
        }
/*
        if (Math.abs(robot.arm.getCurrentPosition()) > Math.abs(armTarget) + 100 || Math.abs(robot.arm.getCurrentPosition()) < Math.abs(armTarget) - 100)
        {
            robot.arm.setTargetPosition(Math.abs(armTarget));
            if (Math.abs(robot.arm.getCurrentPosition()) > Math.abs(armTarget) && armTarget > 0)
            {
                robot.arm.setPower(-.25);
            }
            else if (Math.abs(robot.arm.getCurrentPosition()) < Math.abs(armTarget) && armTarget < 2020)
            {
                robot.arm.setPower(.25);
            }
        }
        else
        {
            robot.arm.setPower(0);
        }
*/

        if (gamepad2.b)
        {
            robot.jewelArm.setPosition(.15);
        }
        else
        {
            robot.jewelArm.setPosition(.5);
        }

        if (gamepad2.dpad_right)
        {
            robot.jewelHand.setPosition(1);
        }
        else
        {
            robot.jewelHand.setPosition(.5);
        }


        if (gamepad2.x)
        {
            robot.boot.setPosition(.8);
        }
        else
        {
            robot.boot.setPosition(0);
        }



        //We have issues with the direction the motor is turning. When the direction was set to forward,
        //the first setPower was negative, and the second was positive, the position went way negative, and the arm went out without stopping.

        //At this version of the code, it goes up and correctly, until it reaches the limit,
        // at which point it goes out on its own.
        //if (currentRelic > totalRelicDistance)
        //{
        //    robot.relicextend.setPower(.15);
        //}
        //else if (currentRelic < .5)
        //{
        //    robot.relicextend.setPower(-.15);
        //}
        //else if ((currentRelic <= totalRelicDistance) && (currentRelic >= 0))
        //{
        //    robot.relicextend.setPower(relicMove/4);
        //}

        if (relicMove > 0) {
            robot.relicextend.setPower(relicMove / 2);
        } else {
            robot.relicextend.setPower(relicMove);
        }


        /*

        if (gamepad2.y && hasExecuted) {
            if (armStateOpen) {
                armStateOpen = Boolean.FALSE;
                clawOffset = -.5;

            }   else {
                armStateOpen = Boolean.TRUE;
                clawOffset = 0;
            }
            hasExecuted = false;
        }

        if (gamepad2.y && hasExecuted)
        {
            armStateOpen = Boolean.TRUE;
            clawOffset = -.20;
            hasExecuted = false;
        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (hasExecuted == false && gamepad2.right_bumper == false){
            // clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO - clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO + clawOffset);
            hasExecuted = true;
        }
        */

        //robot.relicretract.setPower(right);

        // relic WRIST
        if (gamepad2.dpad_down && hasExecuted3) {
            hasExecuted3 = false;

            if(armStateOpen3) {

                armStateOpen3 = false;

                clawOffset = .5;

            }   else {

                armStateOpen3 = true;
                clawOffset = 0;
            }

            //robot.relicHand.setPosition(robot.MID_SERVO + clawOffset);

        }

        // relic WRIST
        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (hasExecuted3 == false && !gamepad2.dpad_down) {
            robot.relicHand.setPosition(clawOffset);
            hasExecuted3 = true;
        }


        // relic HAND
        if (gamepad2.dpad_up && hasExecuted2) {
            hasExecuted2 = false;
            if(armStateOpen2) {

                armStateOpen2 = false;
                clawOffset = .5;

            }   else {

                armStateOpen2 = true;
                clawOffset = -.5;
            }

            //robot.relicHand.setPosition(robot.MID_SERVO + clawOffset);

        }

        // relic HAND
        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (hasExecuted2 == false && gamepad2.dpad_up == false) {
            robot.relicHand2.setPosition(clawOffset);
            hasExecuted2 = true;
        }



        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("arm state", "%s", armStateOpen);
        telemetry.addData("Correction", "%s", correction);
        telemetry.addData("Turn", turn);
        telemetry.addData("Relic arm position: ", currentRelic);
        telemetry.addData("Touch Sensor:", robot.touchSensor.getState());
        telemetry.update();

        //telemetry.addData("Target Position", armTarget);
        //telemetry.addData("Current Position", robot.arm.getCurrentPosition());



    /*
     * Code to run ONCE after the driver hits STOP
     */

    }



    @Override
    public void stop() {
    }
}