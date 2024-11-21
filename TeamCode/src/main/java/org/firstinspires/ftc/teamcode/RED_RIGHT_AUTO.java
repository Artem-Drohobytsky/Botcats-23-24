package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RED_RIGHT_AUTO", group="cvAutos")
//RED TOP STARTING PLACE
public class RED_RIGHT_AUTO extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    public void runOpMode() {

        robot.init(hardwareMap,telemetry);
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera pipeline;

        pipeline = new camera();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        DcMotor fourBar = hardwareMap.dcMotor.get("4bar_motor");
        Servo outtake = hardwareMap.servo.get("4bar_servo");

        CRServo back_left_servo = hardwareMap.crservo.get("back_left_servo");
        CRServo back_right_servo = hardwareMap.crservo.get("back_right_servo");
        CRServo front_left_servo = hardwareMap.crservo.get("front_left_servo");
        CRServo front_right_servo = hardwareMap.crservo.get("front_right_servo");

        fourBar.setTargetPosition(200);
        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int beacon = 0;

        Pose2d startPos = new Pose2d(0, 0, 0);
        Pose2d pLeft = new Pose2d(28, 4, Math.toRadians(-90));
        Pose2d pCenter = new Pose2d(44, 0, Math.toRadians(0));
        Pose2d pRight = new Pose2d(28, -4, Math.toRadians(90));
        Pose2d leftPlacement = new Pose2d(37, -40, Math.toRadians(-90));
        Pose2d centerPlacement = new Pose2d(29, -40, Math.toRadians(-90));
        Pose2d rightPlacement = new Pose2d(21, -40, Math.toRadians(-90));
        Pose2d parking = new Pose2d(14, -40, Math.toRadians(-90));

        TrajectorySequence purpleLeft = drive.trajectorySequenceBuilder(startPos)
                .forward(28)
                .turn(-1 * Math.toRadians(90))
                .back(4)
                .build();

        TrajectorySequence purpleMid = drive.trajectorySequenceBuilder(startPos)
                .forward(34)
                .build();

        TrajectorySequence purpleRight = drive.trajectorySequenceBuilder(startPos)
                .forward(28)
                .turn(Math.toRadians(90))
                .back(4)
                .build();

        TrajectorySequence leftBackboard = drive.trajectorySequenceBuilder(pLeft)
                .lineToLinearHeading(leftPlacement)
                .build();

        TrajectorySequence centerBackboard = drive.trajectorySequenceBuilder(pCenter)
                .forward(6)
                .lineToLinearHeading(centerPlacement)
                .build();

        TrajectorySequence rightBackboard = drive.trajectorySequenceBuilder(pRight)
                .strafeLeft(24)
                .lineToLinearHeading(rightPlacement)
                .build();

        DcMotor fl = null;
        DcMotor fr = null;
        DcMotor bl = null;
        DcMotor br = null;

        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("back_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("front_right_motor");

        // fr.setDirection(DcMotorSimple.Direction.REVERSE);
        // bl.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!opModeIsActive()) {
            if (pipeline.getRedAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleRight);

                beacon = 2;
            } else if (pipeline.getRedAnalysis() == camera.SkystonePosition.LEFT) {
                telemetry.addData("left", "found item in left box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleLeft);

                beacon = 0;
            } else if (pipeline.getRedAnalysis() == camera.SkystonePosition.CENTER) {
                telemetry.addData("center", "found item in center box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleMid);

                beacon = 1;
            }
        }
        waitForStart();
        if (opModeIsActive()) {

            outtake.setPosition(0.5);

            telemetry.addData("img analysis", beacon);
            telemetry.update();

            /*if (pipeline.getRedAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleRight);

                beacon = 2;
            }
            else if (pipeline.getRedAnalysis() == camera.SkystonePosition.LEFT) {
                telemetry.addData("left", "found item in left box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleLeft);

                beacon = 0;
            }
            else if (pipeline.getRedAnalysis() == camera.SkystonePosition.CENTER) {
                telemetry.addData("center", "found item in center box");
                telemetry.update();
                // drive.followTrajectorySequence(purpleMid);

                beacon = 1;
            }*/

            if(beacon == 1) {drive.followTrajectorySequence(purpleMid);}
            if(beacon == 0) {drive.followTrajectorySequence(purpleLeft);}
            if(beacon == 2) {drive.followTrajectorySequence(purpleRight);}
            // drive.followTrajectorySequence(test);

            front_left_servo.setPower(1);
            front_right_servo.setPower(-1);
            back_left_servo.setPower(1);
            back_right_servo.setPower(-1);
            sleep(1000);

            /* if(beacon == 1) {drive.followTrajectorySequence(centerBackboard);}
            if(beacon == 0) {drive.followTrajectorySequence(leftBackboard);}
            if(beacon == 2) {drive.followTrajectorySequence(rightBackboard);}

            fourBar.setTargetPosition(200);
            fourBar.setPower(0.5);

            outtake.setPosition(0.05); */
        }
    }
}