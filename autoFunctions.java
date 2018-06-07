package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.HardwareRecoverer;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Locale;

/**
 * Created by student on 2/21/2018.
 */


public abstract class autoFunctions extends LinearOpMode{

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
    public ElapsedTime runtime = new ElapsedTime();

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

    double ckey = .85;

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parametersView = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    public autoFunctions()
    {

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

        if (robot.isBoardBlue())
        {
            colorTeam = 1;
            colorMult = 1;
            colorTurn = 0;
        }


    }
    public void drive(double speed)
    {
        robot.ldFront.setPower(speed);
        robot.ldBack.setPower(speed);
        robot.rdFront.setPower(speed);
        robot.rdBack.setPower(speed);
    }

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

    public void slideSide(double speed,
                          double leftInches, double rightInches,
                          double timeoutS) {

        int newLeftTarget1;
        int newRightTarget1;
        int newLeftTarget2;
        int newRightTarget2;

        if (opModeIsActive()) {

            newLeftTarget1 = robot.ldFront.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
            newLeftTarget2 = robot.ldBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget1 = robot.rdFront.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.rdBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.ldFront.setTargetPosition(newLeftTarget1);
            robot.ldBack.setTargetPosition(newLeftTarget2);
            robot.rdFront.setTargetPosition(newRightTarget1);
            robot.rdBack.setTargetPosition(newRightTarget2);

            robot.ldFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ldBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rdFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rdBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.ldFront.setPower(Math.abs(speed));
            robot.ldBack.setPower(Math.abs(speed));
            robot.rdFront.setPower(Math.abs(speed));
            robot.rdBack.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.ldFront.isBusy() && robot.rdFront.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget1,  newRightTarget1);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.ldFront.getCurrentPosition(),
                        robot.rdFront.getCurrentPosition());
                telemetry.update();
            }

            robot.ldFront.setPower(0);
            robot.ldBack.setPower(0);
            robot.rdFront.setPower(0);
            robot.rdBack.setPower(0);

            robot.ldFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ldBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rdFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rdBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
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
        if (newHeading < -90 && heading > 90)
        {
            heading = 360 + heading;
        }



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

        while ((heading > newHeading + 1.5 || heading < newHeading - 1.5) && opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if (newHeading > 180 && heading < -90)
            {
                heading = heading + 360;
            }

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
            encoderDrive(DRIVE_SPEED, 3.5, 3.5, 2.0);
        } else {
            encoderDrive(DRIVE_SPEED, 3, 3, 2.0);
            robot.jewelArm.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -3.5, -3.5, 2.0);
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
    public void lookAtTheThing()
    {
        boolean seen = false;
        double counter = 30000;

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
                        ckey = 2.98;
                    }
                }

                //dddsf

                if (vuMark.toString().equals("CENTER"))
                {
                    ckey = 2;

                }
                else if (vuMark.toString().equals("LEFT"))
                {
                    ckey = 2.7;
                    if (robot.isBoardBlue())
                    {
                        ckey = .85;
                    }
                }

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));


                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
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
            else
            {
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

}
