// This is our first try generating an autonomous program
// Hope it works!

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Silver Auto", group="Commodores")
//@Disabled
public class silverAutonomousOpMode extends LinearOpMode
{
    //Sensors
    private BNO055IMU imu;
    private DigitalChannel elevatorExtended;

    //Outputs
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;
    private Servo leftElevatorMotor;
    private Servo rightElevatorMotor;
    private Servo landerServo;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo markerServo;

    private Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // REV 20:1 Planetary Gearbox
    static final double     DRIVE_GEAR_REDUCTION    = .5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     FAST_DRIVE_SPEED        = 1.0;
    static final double     MEDIUM_DRIVE_SPEED      = .75;
    static final double     SLOW_DRIVE_SPEED        = .2;
    static final double     TURN_SLOW               = 0.5;
    static final double     TURN_FAST               = 0.75;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // setup hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        elevatorExtended = hardwareMap.get(DigitalChannel.class, "elevatorExtended");

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        leftElevatorMotor = hardwareMap.get(Servo.class, "leftElevatorMotor");
        rightElevatorMotor = hardwareMap.get(Servo.class, "rightElevatorMotor");
        landerServo = hardwareMap.get(Servo.class, "landerServo");
        leftArmServo = hardwareMap.get(Servo.class,"leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class,"rightArmServo");
        markerServo = hardwareMap.get(Servo.class,"markerServo");

        // setup Motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // brake motors
        //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set digital channels to input mode.
        elevatorExtended.setMode(DigitalChannel.Mode.INPUT);

        // hold robot up
        landerServo.setPosition(0);

        // hold marker up
        markerServo.setPosition(0.5);

        // ensure basket is retracted
        leftArmServo.setPosition(0.17);
        rightArmServo.setPosition(0.2);

        // set gyro parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "BNO055IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.
        while (opModeIsActive())
        {
            // release elevator
            landerServo.setPosition(0.25);
            sleep(1000);

            // lower robot
            leftElevatorMotor.setPosition(1.0);
            rightElevatorMotor.setPosition(0);
            sleep(600);
            leftElevatorMotor.setPosition(0.5);
            rightElevatorMotor.setPosition(0.5);
            sleep(1000);

            encoderDrive(FAST_DRIVE_SPEED, -7, -7, 2.0);
            encoderDrive(FAST_DRIVE_SPEED, 7, 7, 2.0);

            // rotate to unhook
            rotate(-12, TURN_FAST);

            // lower elevator
            leftElevatorMotor.setPosition(0);
            rightElevatorMotor.setPosition(1.0);
            sleep(650);
            leftElevatorMotor.setPosition(0.5);
            rightElevatorMotor.setPosition(0.5);
            sleep(500);

            // rotate to face wall
            rotate(-44, TURN_SLOW);
            sleep(500);

            // drive to wall
            encoderDrive(SLOW_DRIVE_SPEED, -40, -40, 4.0);
            sleep(500);

            // rotate to face depot
            rotate(-50, TURN_SLOW);
            sleep(500);

            // drive to depot
            encoderDrive(SLOW_DRIVE_SPEED, 40, 40, 4.0);
            sleep(500);

            // dump marker
            markerServo.setPosition(0);
            sleep(2000);
            markerServo.setPosition(0.5);

            // drive to park
            encoderDrive(SLOW_DRIVE_SPEED, -52, -52, 4.0);
            sleep(500);

            // sit pretty
            armMotor.setPower(0);
            leftElevatorMotor.setPosition(0.5);
            rightElevatorMotor.setPosition(0.5);
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(30000);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .2;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Drive based on encoder counts
     * @param speed speed to drive
     * @param leftInches how far to drive the left motor
     * @param rightInches how far to drive the right motor
     * @param timeoutS how long to run the command in case of failure
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}