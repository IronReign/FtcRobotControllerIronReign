package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Fetches JPEG frames from the Limelight's MJPEG stream for display on FTC Dashboard.
 *
 * The Limelight appears as an ethernet device at 172.29.0.1 when connected via USB.
 * This class fetches frames from the MJPEG stream on a background thread and makes
 * them available as Bitmaps for dashboard.sendImage().
 *
 * Usage:
 *   LimelightStream stream = new LimelightStream();
 *   stream.start();
 *   // In loop:
 *   Bitmap frame = stream.getLatestFrame();
 *   if (frame != null) dashboard.sendImage(frame);
 *   // On stop:
 *   stream.stop();
 */
public class LimelightStream {

    // Limelight USB-ethernet IP and stream endpoint
    public static final String LIMELIGHT_IP = "172.29.0.1";
    public static final int LIMELIGHT_PORT = 5800;
    public static final String STREAM_PATH = "/stream.mjpg";

    // Thread management
    private Thread streamThread;
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicReference<Bitmap> latestFrame = new AtomicReference<>(null);

    // Stats
    private volatile long lastFrameTime = 0;
    private volatile int framesReceived = 0;
    private volatile int lastConsumedFrame = 0;  // Track which frame was last sent to dashboard
    private volatile String status = "Not started";

    // Configuration
    private int targetFps = 5;  // Limit frame rate to reduce CPU load (default low)
    private long minFrameIntervalMs;

    public LimelightStream() {
        setTargetFps(5);
    }

    /**
     * Set target frame rate. Higher = more CPU usage.
     * @param fps Target frames per second (1-30 recommended)
     */
    public void setTargetFps(int fps) {
        this.targetFps = Math.max(1, Math.min(30, fps));
        this.minFrameIntervalMs = 1000 / this.targetFps;
    }

    /**
     * Start the background stream fetcher thread.
     */
    public void start() {
        if (running.get()) {
            System.out.println("LimelightStream: Already running, ignoring start()");
            return;
        }

        System.out.println("LimelightStream: Starting stream thread...");
        running.set(true);
        consecutiveErrors = 0;
        status = "Starting...";

        streamThread = new Thread(this::streamLoop, "LimelightStream");
        streamThread.setDaemon(true);
        streamThread.start();
        System.out.println("LimelightStream: Thread started");
    }

    /**
     * Stop the background stream fetcher.
     */
    public void stop() {
        running.set(false);
        if (streamThread != null) {
            streamThread.interrupt();
            try {
                streamThread.join(1000);
            } catch (InterruptedException ignored) {}
            streamThread = null;
        }
        status = "Stopped";
    }

    /**
     * Get the most recent frame. May return null if no frame available yet.
     * The returned Bitmap should not be recycled by the caller.
     */
    public Bitmap getLatestFrame() {
        return latestFrame.get();
    }

    /**
     * Check if there's a new frame since the last call to getNewFrame().
     * Use this to avoid sending duplicate frames to dashboard.
     */
    public boolean hasNewFrame() {
        return framesReceived > lastConsumedFrame;
    }

    /**
     * Get the latest frame if it's new, otherwise return null.
     * Marks the frame as consumed so subsequent calls return null until a new frame arrives.
     * This prevents sending the same frame to dashboard multiple times.
     */
    public Bitmap getNewFrame() {
        if (framesReceived > lastConsumedFrame) {
            lastConsumedFrame = framesReceived;
            return latestFrame.get();
        }
        return null;
    }

    /**
     * Check if stream is currently running.
     */
    public boolean isRunning() {
        return running.get();
    }

    /**
     * Get current status string for telemetry.
     */
    public String getStatus() {
        return status;
    }

    /**
     * Get number of frames received since start.
     */
    public int getFramesReceived() {
        return framesReceived;
    }

    /**
     * Get time since last frame in milliseconds.
     */
    public long getTimeSinceLastFrame() {
        if (lastFrameTime == 0) return -1;
        return System.currentTimeMillis() - lastFrameTime;
    }

    // Max frame buffer size to prevent memory issues (2MB should be plenty for a 640x480 JPEG)
    private static final int MAX_FRAME_SIZE = 2 * 1024 * 1024;
    private volatile int consecutiveErrors = 0;
    private static final int MAX_CONSECUTIVE_ERRORS = 10;

    /**
     * Main stream reading loop - runs on background thread.
     */
    private void streamLoop() {
        while (running.get()) {
            // Stop if too many consecutive errors
            if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
                status = "Too many errors, stopped";
                running.set(false);
                break;
            }

            HttpURLConnection connection = null;
            InputStream inputStream = null;

            try {
                // Connect to MJPEG stream
                String urlString = "http://" + LIMELIGHT_IP + ":" + LIMELIGHT_PORT + STREAM_PATH;
                System.out.println("LimelightStream: Connecting to " + urlString);
                URL url = new URL(urlString);
                connection = (HttpURLConnection) url.openConnection();
                connection.setConnectTimeout(3000);
                connection.setReadTimeout(5000);
                connection.setRequestProperty("Connection", "keep-alive");

                System.out.println("LimelightStream: Getting response code...");
                int responseCode = connection.getResponseCode();
                System.out.println("LimelightStream: Response code = " + responseCode);
                if (responseCode != 200) {
                    status = "HTTP " + responseCode;
                    consecutiveErrors++;
                    Thread.sleep(2000);
                    continue;
                }

                status = "Connected";
                consecutiveErrors = 0;  // Reset on successful connect
                System.out.println("LimelightStream: Connected successfully, reading frames...");
                inputStream = new BufferedInputStream(connection.getInputStream(), 16384);

                // Read MJPEG frames
                ByteArrayOutputStream frameBuffer = new ByteArrayOutputStream(65536);
                byte[] buffer = new byte[8192];
                boolean inFrame = false;

                while (running.get()) {
                    int bytesRead = inputStream.read(buffer);
                    if (bytesRead == -1) {
                        status = "Stream ended";
                        break;
                    }

                    // Scan for JPEG markers
                    for (int i = 0; i < bytesRead && running.get(); i++) {
                        int b = buffer[i] & 0xFF;

                        if (!inFrame) {
                            // Look for JPEG start marker (0xFF 0xD8)
                            if (b == 0xFF && i + 1 < bytesRead && (buffer[i + 1] & 0xFF) == 0xD8) {
                                inFrame = true;
                                frameBuffer.reset();
                                frameBuffer.write(0xFF);
                                frameBuffer.write(0xD8);
                                i++; // Skip the 0xD8
                            }
                        } else {
                            // Safety: prevent runaway buffer growth
                            if (frameBuffer.size() > MAX_FRAME_SIZE) {
                                inFrame = false;
                                frameBuffer.reset();
                                continue;
                            }

                            frameBuffer.write(b);

                            // Look for JPEG end marker (0xFF 0xD9)
                            if (b == 0xD9 && frameBuffer.size() > 2) {
                                byte[] frameBytes = frameBuffer.toByteArray();
                                int lastIdx = frameBytes.length - 2;
                                if (lastIdx >= 0 && (frameBytes[lastIdx] & 0xFF) == 0xFF) {
                                    // Complete JPEG frame
                                    processFrame(frameBytes);
                                    inFrame = false;
                                    frameBuffer.reset();

                                    // Rate limiting
                                    long elapsed = System.currentTimeMillis() - lastFrameTime;
                                    if (elapsed < minFrameIntervalMs) {
                                        Thread.sleep(minFrameIntervalMs - elapsed);
                                    }
                                }
                            }
                        }
                    }
                }

            } catch (InterruptedException e) {
                // Normal shutdown
                break;
            } catch (java.net.SocketTimeoutException e) {
                status = "Timeout";
                consecutiveErrors++;
            } catch (java.net.ConnectException e) {
                status = "Cannot connect";
                consecutiveErrors++;
            } catch (Exception e) {
                status = "Error: " + e.getClass().getSimpleName();
                consecutiveErrors++;
            } finally {
                if (inputStream != null) {
                    try { inputStream.close(); } catch (Exception ignored) {}
                }
                if (connection != null) {
                    try { connection.disconnect(); } catch (Exception ignored) {}
                }
            }

            // Back off before retry
            if (running.get() && consecutiveErrors > 0) {
                try {
                    Thread.sleep(Math.min(2000 * consecutiveErrors, 10000));
                } catch (InterruptedException e) {
                    break;
                }
            }
        }
    }

    /**
     * Process a complete JPEG frame.
     */
    private void processFrame(byte[] jpegData) {
        try {
            // Decode JPEG to Bitmap
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inMutable = false;
            options.inPreferredConfig = Bitmap.Config.RGB_565;  // Smaller memory footprint

            Bitmap newFrame = BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length, options);

            if (newFrame != null) {
                // Swap in new frame, recycle old one
                Bitmap oldFrame = latestFrame.getAndSet(newFrame);
                if (oldFrame != null && oldFrame != newFrame) {
                    oldFrame.recycle();
                }

                lastFrameTime = System.currentTimeMillis();
                framesReceived++;
                status = "Streaming (" + framesReceived + " frames)";
            }
        } catch (Exception e) {
            status = "Decode error: " + e.getMessage();
        }
    }
}
