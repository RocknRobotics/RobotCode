package frc.robot.Raspberry;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

public class CameraControl {
    public UsbCamera front;
    public UsbCamera left;
    public UsbCamera right;
    public UsbCamera back;

    public CvSink frontSink;
    public CvSink leftSink;
    public CvSink rightSink;
    public CvSink backSink;

    public Mat frontImg;
    public Mat leftImg;
    public Mat rightImg;
    public Mat backImg;

    public CameraControl(UsbCamera front, UsbCamera left, UsbCamera right, UsbCamera back) {
        this.front = front;
        this.left = left;
        this.right = right;
        this.back = back;

        frontSink = CameraServer.getVideo(this.front);
        leftSink = CameraServer.getVideo(this.left);
        rightSink = CameraServer.getVideo(this.right);
        backSink = CameraServer.getVideo(this.back);

        frontImg = new Mat();
        leftImg = new Mat();
        rightImg = new Mat();
        backImg = new Mat();
    }

    public void setResolutionAll(int width, int height) {
        front.setResolution(width, height);
        left.setResolution(width, height);
        right.setResolution(width, height);
        back.setResolution(width, height);
    }

    public void setFPSAll(int fps) {
        front.setFPS(fps);
        left.setFPS(fps);
        right.setFPS(fps);
        back.setFPS(fps);
    }

    public void update() {
        Thread frontFrame = frameGrabber(frontSink, frontImg);
        Thread leftFrame = frameGrabber(leftSink, leftImg);
        Thread rightFrame = frameGrabber(rightSink, rightImg);
        Thread backFrame = frameGrabber(backSink, backImg);

        frontFrame.start();
        leftFrame.start();
        rightFrame.start();
        backFrame.start();

        for(int i = 0; i < 25 && frontFrame.isAlive() && leftFrame.isAlive() && rightFrame.isAlive() && backFrame.isAlive(); i++) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public Thread frameGrabber(CvSink aSink, Mat aImg) {
        return new Thread(() -> {
            aSink.grabFrame(aImg);
            this.close();
        });
    }

    public void close() {
        front.close();
        left.close();
        right.close();
        back.close();
    }
}
