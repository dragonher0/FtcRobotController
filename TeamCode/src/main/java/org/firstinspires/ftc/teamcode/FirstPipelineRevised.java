package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;


public class FirstPipelineRevised implements VisionProcessor
{
    public Scalar nonSelectedColor = new Scalar(0,255,0);
    public Scalar selectedColor = new Scalar(0,0,255);
    public Rect rectLeft = new Rect(150, 450, 40, 40);
    public Rect rectMiddle = new Rect(800, 450, 40, 40);
    public Rect rectRight = new Rect(1200, 450, 40, 40);
    public int selectedRect = 0;
    Selected select;
    double selected = 0;

    Mat hsvMat = new Mat();
    Mat destMat = new Mat();
    Mat detectionMat = new Mat();
    Mat submat = new Mat();
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        double satLeft = getAvgSaturation(hsvMat, rectLeft);
        double satMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRight = getAvgSaturation(hsvMat, rectRight);
        Core.extractChannel(hsvMat, detectionMat, 1);
        Imgproc.cvtColor(detectionMat, destMat, Imgproc.COLOR_GRAY2RGB);

        if ((satLeft > satMiddle) && (satLeft > satRight)) {
            selected = 1;
            select = Selected.LEFT;
        } else if ((satMiddle > satLeft) && (satMiddle > satRight)) {
            selected = 2;
            select = Selected.MIDDLE;
        } else {
            selected = 3;
            select = Selected.RIGHT;
        }


        // REMOVE DURING PRODUCTION
        drawRectangles(frame, selected);
        // END REMOVE DURING PRODUCTION
        return select;
    }
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    public void drawRectangles(Mat input, double selected)
    {
        Imgproc.rectangle(input, rectLeft, nonSelectedColor);
        Imgproc.rectangle(input, rectMiddle, nonSelectedColor);
        Imgproc.rectangle(input, rectRight, nonSelectedColor);

        if (selected == 1) {
            Imgproc.rectangle(input, rectLeft, selectedColor);
        } else if (selected == 2) {
            Imgproc.rectangle(input, rectMiddle, selectedColor);
        } else if (selected == 3) {
            Imgproc.rectangle(input, rectRight, selectedColor);
        }
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
    public double getSelection() {
        return selected;
    }
}