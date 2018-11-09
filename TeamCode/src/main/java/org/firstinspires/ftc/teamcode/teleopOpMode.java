package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Teleop", group="Commodores")

public class teleopOpMode extends LinearOpMode {
    //Sensors
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

    @Override
    public void runOpMode() {
        // setup hardware
        elevatorExtended = hardwareMap.get(DigitalChannel.class, "elevatorExtended");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        leftElevatorMotor = hardwareMap.get(Servo.class, "leftElevatorMotor");
        rightElevatorMotor = hardwareMap.get(Servo.class, "rightElevatorMotor");
        landerServo = hardwareMap.get(Servo.class, "landerServo");
        leftArmServo = hardwareMap.get(Servo.class,"leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class,"rightArmServo");

        // setup Motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // brake motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set digital channels to input mode.
        elevatorExtended.setMode(DigitalChannel.Mode.INPUT);

        // hold robot up
        landerServo.setPosition(0);

        // ensure basket is retracted
        leftArmServo.setPosition(0.17);
        rightArmServo.setPosition(0.2);

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // setup speed variables
        double leftPower = 0;
        double rightPower = 0;
        double armPower = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // driving is from gamepad1 sticks
            leftPower = -this.gamepad1.right_stick_y;
            rightPower = -this.gamepad1.left_stick_y;
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // arm is controlled by the gamepad2 triggers
            armPower = gamepad2.right_trigger * .6;
            // if the left trigger is pressed, reverse
            if (gamepad2.left_trigger != 0.0 && gamepad2.right_trigger == 0.0) {
                armPower = -gamepad2.left_trigger * .6;
            }
            armMotor.setPower(armPower);

            //elevator is controlled by gamepad2 bumpers
            if (gamepad2.right_bumper && !gamepad2.left_bumper && elevatorExtended.getState()) {
                leftElevatorMotor.setPosition(0);
                rightElevatorMotor.setPosition(1.0);
            }
            else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                leftElevatorMotor.setPosition(1.0);
                rightElevatorMotor.setPosition(0.0);
            }
            else {
                leftElevatorMotor.setPosition(0.5);
                rightElevatorMotor.setPosition(0.5);
            }

            // latch for elevator
            if (gamepad1.right_bumper) {
                landerServo.setPosition(0);
            } else {
                landerServo.setPosition(0.25);
            }

            // extend the basket
            if (gamepad2.dpad_up){
                leftArmServo.setPosition(.8);
                rightArmServo.setPosition(.8);
            } else if (gamepad2.dpad_down){
                leftArmServo.setPosition(0.17);
                rightArmServo.setPosition(0.2);
            }

            // view sensor and motor states
            telemetry.addData("Elevator Extended Limit: ", elevatorExtended.getState());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}