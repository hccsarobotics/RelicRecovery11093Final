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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RecovererAuto", group="Pushbot")

public class RecovererAuto extends LinearOpMode {


    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    /* Declare OpMode members. */
    HardwareRecoverer robot   = new HardwareRecoverer();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.175;
    static final double     TURN_SPEED              = 0.175;

    double heading;
    double headingAdjuster;
    double directionTo;
    double colorTeam = -1;
    double colorMult = 0;
    double colorTurn = 1;

    double ckey = 1;

    //static final double     BLUE_THRESHOLD          =  30;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersView = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        telemetry.addData("Board Sensor Val ", robot.boardColorSensor.blue());
        if (robot.boardColorSensor.red() < robot.boardColorSensor.blue()) {
            robot.setBoardBlue(Boolean.TRUE);
            telemetry.addData("  ", "BLUE TEAM");
        } else {

            robot.setBoardBlue(Boolean.FALSE);
            telemetry.addData("   ", "RED TEAM");

        }

        parametersView.vuforiaLicenseKey = "AQjggZX/////AAAAGXGdvmEbx0YhlN/Ij3zVV295T2U9JTX+nCKRQn0mXT2dzMmWDQ5IE1a2Erm4JruMaArvBftR6BuuQwAmNv8tov7ThZBhes5cB6fHFghsP9K3uW+rVGIXikKLUX+AtN+TyrM+o7ajnAmd/s7wbrU7hKY9K29IDGkekkJpCOYhSBIY06KMcqdxxy+0crafgK9O48Q6s2r+dDOy2q1+xrW6NOEPZyC/1Lgx9H0hE2Xpm9tNhVhQtRxC40w7JV9CJmzXwrY+Xssuu0KDPg3fGFqTsUiAMaDXfT+TKvGIHBrZ5Fsf/dMMonmM3zvsQHzlMZdArh+hblAsUmZSDWmzlMGjeLNmUmW3qxEn+TE5RmE6hqjP";

        parametersView.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersView);

        robot.ldFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rdFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ldBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rdBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.ldFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ldBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rdFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rdBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                            robot.ldFront.getCurrentPosition(),
                            robot.ldFront.getCurrentPosition(),
                            robot.ldFront.getCurrentPosition(),
                            robot.ldBack.getCurrentPosition());

        telemetry.addData("I see ", robot.ballColorSensor.blue());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //headingAdjuster = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        if (robot.isBoardBlue())
        {
            colorTeam = 1;
            colorMult = 1;
            colorTurn = 0;
        }

        //clawOpen(false); Reidy boy luvs roboot
        //sleep(500);


        // When the board is red, colorTeam is -1, colorMult is 0, and colorTurn is 1.

        // When the board is blue, colorTeam is 1, colorMult is 1, and colorTurn is 0

        // From the first direction, a right turn is -90, and a left turn is 90
        lift("up", 1500);
        sleep(500);
        hitBall();
        encoderDrive(DRIVE_SPEED,  -5,  -5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        lookAtTheThing();
        sleep(500);
        telemetry.update();
        encoderDrive(DRIVE_SPEED,  20*colorTeam + (20 * colorMult),  20*colorTeam + (20 * colorMult), 5.0);
        twistIt(90);
        encoderDrive(DRIVE_SPEED, -6.5, -6.5, 3.0);
        encoderDrive(DRIVE_SPEED,  (-6*ckey) - ckey,  (-6*ckey) - ckey, 6.0); //On This line I removed the color team variable.
        twistIt(180*(colorMult)); //REid needa brake
        //lift("down", 1300);
        //clawOpen(true);
        encoderDrive(DRIVE_SPEED, -5, -5, 4.0);
        boot(true);
        sleep(500);
        encoderDrive(DRIVE_SPEED, 8, 8, 4.0);

        /*

        clawOpen(false);
        encoderDrive(DRIVE_SPEED, 25, 25, 6);
        clawOpen(true);
*/
        /*
        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
        robot.arm.setPower(robot.ARM_UP_POWER);
        sleep(500);
        robot.arm.setPower(robot.ARM_DOWN_POWER);
        sleep(500);
        robot.arm.setPower(0);
        */

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        rightInches = rightInches*-1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.ldFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.ldBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.ldFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.rdBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.ldFront.setTargetPosition(newLeftTarget);
            robot.ldBack.setTargetPosition(newLeftTarget);
            robot.rdFront.setTargetPosition(newRightTarget);
            robot.rdBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.ldFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ldBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rdFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rdBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.ldFront.setPower(Math.abs(speed));
            robot.ldBack.setPower(Math.abs(speed));
            robot.rdFront.setPower(Math.abs(speed));
            robot.rdBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.ldFront.isBusy() && robot.rdFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.ldFront.getCurrentPosition(),
                                            robot.rdFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.ldFront.setPower(0);
            robot.ldBack.setPower(0);
            robot.rdFront.setPower(0);
            robot.rdBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.ldFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ldBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rdFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rdBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void twistIt(double newHeading){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));


        /*
        if (directionTo < 0)
        {
            directionTo = 360 + directionTo;
        }
        */


        //newHeading = (newHeading - headingAdjuster) % 360;

        if (newHeading - heading < 1)
        {
            robot.ldFront.setPower(TURN_SPEED);
            robot.ldBack.setPower(TURN_SPEED);
            robot.rdFront.setPower(TURN_SPEED);
            robot.rdBack.setPower(TURN_SPEED);
        }
        else if (newHeading - heading > 0)
        {
            robot.ldFront.setPower(-TURN_SPEED);
            robot.ldBack.setPower(-TURN_SPEED);
            robot.rdFront.setPower(-TURN_SPEED);
            robot.rdBack.setPower(-TURN_SPEED);
        }

        telemetry.addData("DirectionTo", directionTo);

        while ((heading > newHeading + 2.5 || heading < newHeading - 2.5) && opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            /*
            telemetry
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    });
                    */
        }
        robot.ldFront.setPower(0);
        robot.ldBack.setPower(0);
        robot.rdFront.setPower(0);
        robot.rdBack.setPower(0);

    }
    public void hitBall() {
        robot.jewelArm.setPosition(.5);

        sleep(1500);

        double ballColor = robot.ballColorSensor.blue();
        if (ballColor > robot.ballColorSensor.red()) {
            robot.setBallBlue(Boolean.TRUE);

        } else {
            robot.setBallBlue(Boolean.FALSE);
        }

        if (robot.isBoardBlue() ^ robot.isBallBlue()) {
            encoderDrive(DRIVE_SPEED, -3, -3, 2.0);
            robot.jewelArm.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, 4, 4, 2.0);
        } else {
            encoderDrive(DRIVE_SPEED, 3, 3, 2.0);
            robot.jewelArm.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -4, -4, 2.0);
        }

        if (robot.isBallBlue()) {
            telemetry.addData("  I see the blue ball  ", ballColor);
        } else {
            telemetry.addData("  I see the red ball  ", ballColor);
        }

        telemetry.update();
        sleep(1500);
    }
    public void clawOpen(boolean open){
        if (open)
        {
            robot.rightClaw.setPosition(.4);
            robot.leftClaw.setPosition(1);
        }
        else
        {
            robot.rightClaw.setPosition(1);
            robot.leftClaw.setPosition(.4);
        }

    }
    public void lift(String power, int wait)
    {
        if (power == "up")
        {
            robot.arm.setPower(robot.ARM_UP_POWER);
            sleep(wait);
            robot.arm.setPower(0);
        }
        else if (power == "down")
        {
            robot.arm.setPower(robot.ARM_DOWN_POWER);
            sleep(wait);
            robot.arm.setPower(0);
        }
    }

    public void lookAtTheThing()
    {
        boolean seen = false;
        double counter = 2500;
        boolean movedMore = false;

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        while (seen == false) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            relicTrackables.activate();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                if (vuMark.toString().equals("RIGHT"))
                {
                    ckey = 1;
                    if (robot.isBoardBlue())
                    {
                        ckey = 3;
                    }
                }

                if (vuMark.toString().equals("CENTER"))
                {
                    ckey = 2;

                }
                else if (vuMark.toString().equals("LEFT"))
                {
                    ckey = 3;
                    if (robot.isBoardBlue())
                    {
                        ckey = 1;
                    }
                }

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));


                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (movedMore)
                {
                    encoderDrive(DRIVE_SPEED, 3, 3, 5.0);
                }
                /*
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                */
                telemetry.addData("CKey is ", ckey);
                telemetry.update();
                seen = true;

            } else if (counter > 0)
            {
                telemetry.addData("Not", "Visible");
                telemetry.update();
                counter = counter - 1;
            }
            else if (movedMore == false)
            {
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                movedMore = true;
            }
            else
            {
                if (movedMore)
                {
                    encoderDrive(DRIVE_SPEED, 3, 3, 5.0);
                }
                seen = true;
            }
        }

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void boot(boolean kicking)
    {
        if (kicking)
        {
            robot.boot.setPosition(.8);
        }
        else
        {
            robot.boot.setPosition(0);
        }

    }
}
