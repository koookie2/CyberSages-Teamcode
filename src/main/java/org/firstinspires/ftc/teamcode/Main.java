package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private DcMotorEx front_right_motor;
    private DcMotorEx front_left_motor;
    private DcMotorEx back_left_motor;
    private DcMotorEx back_right_motor;
    private DcMotorEx arm;
    private DcMotorEx viper;
    private Servo picking;
    private WebcamName webcam;

    private double robotSpeed;


    @Override
    public void runOpMode() {
        setMotorsAndServos();
        setSpecifications();

        robotSpeed = 0.75;

        waitForStart();

        while (driveTrainIsBusy()) print();
        while (opModeIsActive()) {
            if (gamepad1.a) robotSpeed = 0.80125;
            if (gamepad1.b) robotSpeed = 0.60;
            if (gamepad1.y) robotSpeed = 0.3367;
            if (gamepad1.x) robotSpeed = 0.25;

            move();
            strafe();
            // turn();
            armFunction();
            print();
        }
    }

    private void armFunction() {
        // arm movement
        if (gamepad2.right_trigger > 0 && arm.getCurrentPosition() < 3040) arm.setPower(0.8);
        else if (gamepad2.left_trigger > 0 && arm.getCurrentPosition() > 0) arm.setPower(-0.6);
        else arm.setPower(0);

        // viper movement
        if (gamepad2.right_bumper && viper.getCurrentPosition() < 2150) viper.setPower(1);
        else if (gamepad2.left_bumper && viper.getCurrentPosition() > 75) viper.setPower(-1);
        else viper.setPower(0);

        // Servo movement
        if (gamepad2.a) picking.setPosition(0);
        else picking.setPosition(0.28);

        // Hanging macro
        if (gamepad2.dpad_down) {
            front_right_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            front_left_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            back_right_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            back_left_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            front_right_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            front_left_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            back_right_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            back_left_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            viper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            front_right_motor.setTargetPosition(0);
            front_left_motor.setTargetPosition(0);
            back_right_motor.setTargetPosition(0);
            back_left_motor.setTargetPosition(0);
            arm.setTargetPosition(0);
            viper.setTargetPosition(0);

            front_right_motor.setTargetPositionTolerance(10);
            front_left_motor.setTargetPositionTolerance(10);
            back_right_motor.setTargetPositionTolerance(10);
            back_left_motor.setTargetPositionTolerance(10);
            arm.setTargetPositionTolerance(10);
            viper.setTargetPositionTolerance(10);

            runDriveTrain(-400, 0.5);
            run(arm, 2450, 1);
            while (opModeIsActive() && arm.isBusy()) print();

            run(viper, 2200, 0.9);

            while (opModeIsActive() && viper.isBusy()) print();

            run(arm, 0, 0.3);
            run(viper, 500, 0.6);

            while (opModeIsActive() && arm.isBusy()) print();

            run(viper, 1500, 0.5);
            run(arm, -100, 0.5);

            while (opModeIsActive() && viper.isBusy()) print();

            run(arm, 500, 0.3);

            while (opModeIsActive() && arm.isBusy()) print();

            run(viper, 490, 0.3);
            run(arm, 0, 0.3);

            while (opModeIsActive() && (arm.isBusy() || viper.isBusy())) print();
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

    // private void turn() {
    //     if (gamepad1.left_bumper) {
    //         front_right_motor.setPower(0.5*robotSpeed);
    //         front_left_motor.setPower(-0.5*robotSpeed);
    //         back_right_motor.setPower(0.5*robotSpeed);
    //         back_left_motor.setPower(-0.5*robotSpeed);
    //     }
    //     if (gamepad1.right_bumper) {
    //         front_right_motor.setPower(-0.5*robotSpeed);
    //         front_left_motor.setPower(0.5*robotSpeed);
    //         back_right_motor.setPower(-0.5*robotSpeed);
    //         back_left_motor.setPower(0.5*robotSpeed);
    //     }
    // }

    private void move() {
        front_right_motor.setPower(-1 * robotSpeed * gamepad1.left_stick_y + -0.75 * robotSpeed * gamepad1.right_stick_x);
        front_left_motor.setPower(-1 * robotSpeed * gamepad1.left_stick_y + 0.75 * robotSpeed * gamepad1.right_stick_x);
        back_left_motor.setPower(-1 * robotSpeed * gamepad1.left_stick_y + 0.75 * robotSpeed * gamepad1.right_stick_x);
        back_right_motor.setPower(-1 * robotSpeed * gamepad1.left_stick_y + -0.75 * robotSpeed * gamepad1.right_stick_x);
    }

    // private void turn() {
    //     front_right_motor.setPower(-1 * robotSpeed * gamepad1.right_stick_x);
    //     front_left_motor.setPower(-1 * robotSpeed * gamepad1.right_stick_x);
    //     back_left_motor.setPower(-1 * robotSpeed * gamepad1.right_stick_x);
    //     back_right_motor.setPower(-1 * robotSpeed * gamepad1.right_stick_x);
    // }

    private void strafe() {
        if (gamepad1.right_bumper) {
            front_right_motor.setPower(-robotSpeed);
            back_right_motor.setPower(robotSpeed);
            back_left_motor.setPower(-robotSpeed);
            front_left_motor.setPower(robotSpeed);
        } else if (gamepad1.left_bumper) {
            front_right_motor.setPower(robotSpeed);
            back_right_motor.setPower(-robotSpeed);
            back_left_motor.setPower(robotSpeed);
            front_left_motor.setPower(-robotSpeed);
        }
    }

    private void setMotorsAndServos() {
        front_right_motor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        front_left_motor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        back_right_motor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        back_left_motor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        viper = hardwareMap.get(DcMotorEx.class, "viper");
        picking = hardwareMap.get(Servo.class, "picking");
        webcam = hardwareMap.get(WebcamName.class, "Webcam1");
    }

    private void setSpecifications() {
        front_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        viper.setDirection(DcMotorEx.Direction.REVERSE);

        setIndividualSpecifications(front_right_motor);
        setIndividualSpecifications(front_left_motor);
        setIndividualSpecifications(back_right_motor);
        setIndividualSpecifications(back_left_motor);
        setIndividualSpecifications(arm);
        setIndividualSpecifications(viper);
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
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("viper", viper.getCurrentPosition());
        telemetry.update();
    }
}
