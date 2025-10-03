package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private DcMotorEx front_right_motor;
    private DcMotorEx front_left_motor;
    private DcMotorEx back_left_motor;
    private DcMotorEx back_right_motor;
    private double robotSpeed;

    @Override
    public void runOpMode() {
        setMotorsAndServos();
        setSpecifications();

        robotSpeed = 0.75;

        waitForStart();

        // while (driveTrainIsBusy()) print();
        while (opModeIsActive()) {
            if (gamepad1.a) robotSpeed = 0.80125;
            if (gamepad1.b) robotSpeed = 0.60;
            if (gamepad1.y) robotSpeed = 0.3367;
            if (gamepad1.x) robotSpeed = 0.25;

            move();
            print();
        }
    }

    private boolean driveTrainIsBusy() {
        return opModeIsActive() && (front_right_motor.isBusy() || front_left_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy());
    }

    private void run(DcMotorEx motor, double targetPosition, double power) {
        motor.setTargetPosition((int) targetPosition);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void runDriveTrain(double targetPosition, double power) {
        runDriveTrain(targetPosition, targetPosition, targetPosition, targetPosition, power);
    }

    private void runDriveTrain(double front_right_targetPosition, double front_left_targetPosition, double back_right_targetPosition, double back_left_targetPosition, double power) {
        run(front_right_motor, front_right_motor.getTargetPosition() + front_right_targetPosition, power);
        run(front_left_motor, front_left_motor.getTargetPosition() + front_left_targetPosition, power);
        run(back_right_motor, back_right_motor.getTargetPosition() + back_right_targetPosition, power);
        run(back_left_motor, back_left_motor.getTargetPosition() + back_left_targetPosition, power);
    }

    private void move() {
        front_right_motor.setPower(vertical() + horizontal("up") + turn("right"));
        front_left_motor.setPower(vertical() + horizontal("down") + turn("left"));
        back_right_motor.setPower(vertical() + horizontal("down") + turn("right"));
        back_left_motor.setPower(vertical() + horizontal("up") + turn("left"));
    }

    private double vertical() {
        return robotSpeed * gamepad1.left_stick_y;
    }

    private double horizontal(String diagonal) {
        if (diagonal.equals("up")) {
            return -robotSpeed * gamepad1.right_stick_x;
        } else {
            return robotSpeed * gamepad1.right_stick_x;
        }
    }

    private double turn(String side) {
        if (side.equals("right")) {
            return gamepad1.left_bumper? robotSpeed: gamepad1.right_bumper? -robotSpeed: 0;
        } else {
            return gamepad1.left_bumper? -robotSpeed: gamepad1.right_bumper? robotSpeed: 0;
        }
    }

    private void setMotorsAndServos() {
        front_right_motor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        front_left_motor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        back_right_motor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        back_left_motor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
    }

    private void setSpecifications() {
        front_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorEx.Direction.REVERSE);

        setIndividualSpecifications(front_right_motor);
        setIndividualSpecifications(front_left_motor);
        setIndividualSpecifications(back_right_motor);
        setIndividualSpecifications(back_left_motor);
    }

    private void setIndividualSpecifications(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(0);
        motor.setTargetPositionTolerance(10);
    }

    private void print() {
        telemetry.addData("front_right_motor", front_right_motor.getCurrentPosition());
        telemetry.addData("front_left_motor", front_left_motor.getCurrentPosition());
        telemetry.addData("back_right_motor", back_right_motor.getCurrentPosition());
        telemetry.addData("back_left_motor", back_left_motor.getCurrentPosition());
        telemetry.update();
    }
}