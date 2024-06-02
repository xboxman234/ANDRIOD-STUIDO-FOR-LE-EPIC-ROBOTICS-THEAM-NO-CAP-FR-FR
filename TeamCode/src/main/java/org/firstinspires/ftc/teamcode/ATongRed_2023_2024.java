package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Base64;

@Autonomous
public  class ATongRed_2023_2024  extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    public DcMotor Leftback;
    public DcMotor Leftfront;
    public DcMotor Rightback;
    public DcMotor Rightfront;
    public DcMotor Extender;
    public DcMotor Rotator;
    //public Servo Clasp;
    public Servo YellowPixel;
    public Servo PurplePixel;
    public double TicksToMoves;
    @Override
    public void runOpMode()
    {
        Extender = hardwareMap.get(DcMotor.class, "Extender");
        Rotator = hardwareMap.get(DcMotor.class, "Rotator");
        Leftback = hardwareMap.get(DcMotor.class, "Leftback");
        Leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        Rightback = hardwareMap.get(DcMotor.class, "Rightback");
        Rightfront = hardwareMap.get(DcMotor.class, "Rightfront");
        YellowPixel =hardwareMap.get(Servo.class, "YellowPixel");
        PurplePixel =hardwareMap.get(Servo.class, "PurplePixel");
        Leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rightback.setDirection(DcMotor.Direction.REVERSE);
        Rightfront.setDirection(DcMotor.Direction.REVERSE);
        Leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(2560,1440, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("error:",errorCode);
                telemetry.update();
            }
        });
        YellowPixel.setPosition(0.2);
        PurplePixel.setPosition(0.04);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                //Encoder_Move(3, 0.33, "Backward");//move 3in back
                telemetry.addData("somthing:",SkystoneDeterminationPipeline.hi);
                sleep(1000);
                if (SkystoneDeterminationPipeline.hi==1){
                    Encoder_Move(20, 0.33, "Forward");//move 2in back
                    Degree_Turn(126,0.33,"Left");
                    //Encoder_Move(3 , 0.33, "Left");
                    PurplePixel.setPosition(0.65);
                    sleep(1000);
                    PurplePixel.setPosition(0.06);



                    Encoder_Move(20,0.33,"Left");
                    Encoder_Move(75,0.4,"Forward");

                    Encoder_Move(9,0.33,"Right");
                    Encoder_Move(17,0.33,"Forward");
                    //Degree_Turn(135,0.33,"Left");
                    //Encoder_Move(18,0.33,"Forward");


                    YellowPixel.setPosition(0.85);
                    sleep(2000);
                    Encoder_Move(5,0.33,"Backward");
                    YellowPixel.setPosition(0.4);

                    sleep(99999);

                };
                telemetry.update();
                if (SkystoneDeterminationPipeline.hi==2){


                    Encoder_Move(26, 0.33, "Forward");//move 2in back
                    Encoder_Move(3,0.33,"Left");
                    //Encoder_Move(2, 0.33, "Left");
                    PurplePixel.setPosition(0.65);
                    sleep(1000);
                    PurplePixel.setPosition(0.04);
                    Encoder_Move(3,0.33,"Right");
                    Encoder_Move(23, 0.33, "Backward");
                    Degree_Turn(132,0.33,"Left");
                    Encoder_Move(75,0.33,"Forward");

                    Encoder_Move(19 ,0.33,"Right");
                    Encoder_Move(20,0.33,"Forward");
                    //Degree_Turn(135,0.33,"Left");
                    //Encoder_Move(18,0.33,"Forward");


                    YellowPixel.setPosition(0.85);
                    sleep(2000);
                    Encoder_Move(5, 0.1,"Backward");
                    YellowPixel.setPosition(0.4);


                    sleep(99999);



                };
                telemetry.update();
                if (SkystoneDeterminationPipeline.hi==3){

                    //if(object in front){
                    Encoder_Move(16, 0.33, "Forward");//move 2in back
                    Encoder_Move(4.5, 0.33, "Right");
                    PurplePixel.setPosition(0.65);
                    sleep(1500);
                    PurplePixel.setPosition(0.04);
                    Encoder_Move(6,0.33,"Left");
                    Encoder_Move(11,0.33,"Backward");
                    Degree_Turn(130,0.33,"Left");
                    Encoder_Move(68,0.4,"Forward");

                    Encoder_Move(24,0.33,"Right");
                    Encoder_Move(22 ,0.33,"Forward");



                    YellowPixel.setPosition(0.85);
                    sleep(2000);
                    Encoder_Move(5, 0.1,"Backward");
                    sleep(1000);
                    YellowPixel.setPosition(0.1);


                    sleep(99999);
                };
                telemetry.update();
            }
        }

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        public static int hi=0;

        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {

            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0 , 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 490);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(900,330);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1680,620);
        static final int REGION_WIDTH = 400;
        static final int REGION_HEIGHT = 400;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;
        private volatile SkystonePosition position = SkystonePosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                hi=1;

                position = SkystonePosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                hi=2;
                position = SkystonePosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                hi=3;
                position = SkystonePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SkystonePosition getAnalysis()
        {
            return position;
        }
    }
    public void Degree_Turn(double Degrees, double MotorPower, String MovementMode) {
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TicksToMoves = Double.parseDouble(JavaUtil.formatNumber(((Degrees * 0.1828) / (3.78 * Math.PI)) * 537.7, 0));
        if (MovementMode.equals("Left")) {
            Rightfront.setTargetPosition((int) -TicksToMoves);
            Leftfront.setTargetPosition((int) TicksToMoves);
            Rightback.setTargetPosition((int) -TicksToMoves);
            Leftback.setTargetPosition((int) TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(-MotorPower);
                Leftfront.setPower(MotorPower);
                Rightback.setPower(-MotorPower);
                Leftback.setPower(MotorPower);
            }
        } else if (MovementMode.equals("Right")) {
            Rightfront.setTargetPosition((int) TicksToMoves);
            Leftfront.setTargetPosition((int) -TicksToMoves);
            Rightback.setTargetPosition((int) TicksToMoves);
            Leftback.setTargetPosition((int) -TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(MotorPower);
                Leftfront.setPower(-MotorPower);
                Rightback.setPower(MotorPower);
                Leftback.setPower(-MotorPower);
            }
        }
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Encoder_Move(double DistanceToTravel, double MotorPower, String MovementMode) {
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TicksToMoves = Double.parseDouble(JavaUtil.formatNumber((DistanceToTravel / (3.78 * Math.PI)) * 537.7, 0));
        if (MovementMode.equals("Forward")) {
            Rightfront.setTargetPosition((int) TicksToMoves);
            Leftfront.setTargetPosition((int) TicksToMoves);
            Rightback.setTargetPosition((int) TicksToMoves);
            Leftback.setTargetPosition((int) TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(MotorPower);
                Leftfront.setPower(MotorPower);
                Rightback.setPower(MotorPower);
                Leftback.setPower(MotorPower);
            }
        } else if (MovementMode.equals("Backward")) {
            Rightfront.setTargetPosition((int) -TicksToMoves);
            Leftfront.setTargetPosition((int) -TicksToMoves);
            Rightback.setTargetPosition((int) -TicksToMoves);
            Leftback.setTargetPosition((int) -TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(-MotorPower);
                Leftfront.setPower(-MotorPower);
                Rightback.setPower(-MotorPower);
                Leftback.setPower(-MotorPower);
            }
        } else if (MovementMode.equals("Right")) {
            Rightfront.setTargetPosition((int) -TicksToMoves);
            Leftfront.setTargetPosition((int) TicksToMoves);
            Rightback.setTargetPosition((int) TicksToMoves);
            Leftback.setTargetPosition((int) -TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(-MotorPower);
                Leftfront.setPower(MotorPower);
                Rightback.setPower(MotorPower);
                Leftback.setPower(-MotorPower);
            }
        } else if (MovementMode.equals("Left")) {
            Rightfront.setTargetPosition((int) TicksToMoves);
            Leftfront.setTargetPosition((int) -TicksToMoves);
            Rightback.setTargetPosition((int) -TicksToMoves);
            Leftback.setTargetPosition((int) TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(MotorPower);
                Leftfront.setPower(-MotorPower);
                Rightback.setPower(-MotorPower);
                Leftback.setPower(MotorPower);
            }
        } else if (MovementMode.equals("Clockwise")) {
            Rightfront.setTargetPosition((int) -TicksToMoves);
            Leftfront.setTargetPosition((int) TicksToMoves);
            Rightback.setTargetPosition((int) -TicksToMoves);
            Leftback.setTargetPosition((int) TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(-MotorPower);
                Leftfront.setPower(MotorPower);
                Rightback.setPower(-MotorPower);
                Leftback.setPower(MotorPower);
            }
        } else if (MovementMode.equals("Counterclockwise")) {
            Rightfront.setTargetPosition((int) TicksToMoves);
            Leftfront.setTargetPosition((int) -TicksToMoves);
            Rightback.setTargetPosition((int) TicksToMoves);
            Leftback.setTargetPosition((int) -TicksToMoves);
            while (Leftback.isBusy() || Leftfront.isBusy() || Rightback.isBusy() || Rightfront.isBusy()) {
                Rightfront.setPower(MotorPower);
                Leftfront.setPower(-MotorPower);
                Rightback.setPower(MotorPower);
                Leftback.setPower(-MotorPower);
            }
        }
        Leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Arm_Extension(double Extension, double ExtensionPower, String ExtensionMode) {
        Extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TicksToMoves = Double.parseDouble(JavaUtil.formatNumber((Extension / (1.27 * Math.PI)) * 537.7, 0));
        if (ExtensionMode == "Out") {
            Extender.setTargetPosition((int) TicksToMoves);
            while (Extender.isBusy()) {
                Extender.setPower(ExtensionPower);
            }
        } else if (ExtensionMode == "In") {
            Extender.setTargetPosition((int) -TicksToMoves);
            while (Extender.isBusy()) {
                Extender.setPower(-ExtensionPower);
            }
        }
        Extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Arm_Rotation(double Rotation, double RotationPower, String RotationMode) {
        Rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TicksToMoves = Double.parseDouble(JavaUtil.formatNumber((Rotation / 360) * (1425.1/0.15), 0));
        if (RotationMode == "Forward") {
            Rotator.setTargetPosition((int) TicksToMoves);
            while (Rotator.isBusy()) {
                Rotator.setPower(RotationPower);
            }
        } else if (RotationMode == "Back") {
            Rotator.setTargetPosition((int) TicksToMoves);
            while (Rotator.isBusy()) {
                Rotator.setPower(RotationPower);
            }
        }
        Rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
