package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Created by dkrider on 10/27/2017.
 */

@I2cSensor(name = "ZX Distance", description = "Sparkfun ZX Distance Sensor", xmlTag = "ZXDistance")
public class ZXDistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public static final int DEF_I2CADDRESS = 0x20;

    //
    // GestureSense XZ01 Sensor I2C Register Map Version 1.
    //
    private static final int REG_STATUS = 0x00;     //Sensor and Gesture Status
    private static final int REG_DRE = 0x01;     //Data Ready Enable Bitmap
    private static final int REG_DRCFG = 0x02;     //Data Ready Configuration
    private static final int REG_GESTURE = 0x04;     //Last Detected Gesture
    private static final int REG_GSPEED = 0x05;     //Last Detected Gesture Speed
    private static final int REG_DCM = 0x06;     //Data Confidence Metric
    private static final int REG_XPOS = 0x08;     //X Coordinate
    private static final int REG_ZPOS = 0x0a;     //Z Coordinate
    private static final int REG_LRNG = 0x0c;     //Left Emitter Ranging Data
    private static final int REG_RRNG = 0x0e;     //Right Emitter Ranging Data
    private static final int REG_REGVER = 0xfe;     //Register Map Version
    private static final int REG_MODEL = 0xff;     //Sensor Model ID

    //
    // Register 0x00 - STATUS:
    //  DAV - Position Data Available (RO).
    //      1 indicates that new position value is available in the coordinate registers.
    //      This bit automatically resets to zero after being read.
    //  OVF - Brightness value overflow (RO).
    //      Currently unused, reads 0.
    //  SWP - Swipe Gesture Available (RO).
    //      1 indicates that a swipe gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being read.
    //  HOVER - Hover Gesture Available (RO).
    //      1 indicates that a hover gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being readn
    //  HVG - Hover-Move Gesture Available (RO).
    //      1 indicates that a hover-and-move gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being read.
    //  EDGE - Edge Detection Event (RO).
    //      Currently unused, reads 0.
    //  HB - Heartbeat (RO).
    //      This bit will toggle every time the status register has been read.
    //
    public static final int STATUS_DAV = (1 << 0);
    public static final int STATUS_OVF = (1 << 1);
    public static final int STATUS_SWP = (1 << 2);
    public static final int STATUS_HOVER = (1 << 3);
    public static final int STATUS_HVG = (1 << 4);
    public static final int STATUS_EDGE = (1 << 5);
    public static final int STATUS_HB = (1 << 7);
    public static final int STATUS_GESTURES = (STATUS_SWP | STATUS_HOVER | STATUS_HVG);

    //
    // Register 0x01 - DRE (Data Ready Enable):
    //  A '1' in any of these bits will allow the DR pin to assert when the respective event or gesture occurs. The
    //  default value of this register is 0x00, meaning that nothing will cause the DR pin to assert. The value of
    //  this register does not prevent gestures or events from being detected. It only controls which gestures or
    //  events will cause the DR pin to assert.
    //  RNG - Ranging Data Available (RW).
    //      Enable 1 = assert DR when new ranging value is available.
    //  CRD - Coordinate Data Available (RW).
    //      Enable 1 = assert DR when new coordinate value is available.
    //  SWP - Swipe Gestures (RW).
    //      Enable 1 = assert DR when swipe gestures are detected.
    //  HOVER - Hover Gestures (RW).
    //      Enable 1 = assert DR when hover gestures are detected.
    //  HVG - Hover-Move Gestures (RW).
    //      Enable 1 = assert DR when "hover-move" gestures are detected.
    //  EDGE - Edge Detection Events (RW).
    //      Enable 1 = assert DR when edge detection occurs.
    //
    private static final int DRE_RNG = (1 << 0);
    private static final int DRE_CRD = (1 << 1);
    private static final int DRE_SWP = (1 << 2);
    private static final int DRE_HOVER = (1 << 3);
    private static final int DRE_HVG = (1 << 4);
    private static final int DRE_EDGE = (1 << 5);
    private static final int DRE_ALL = (DRE_RNG | DRE_CRD | DRE_SWP | DRE_HOVER | DRE_HVG | DRE_EDGE);

    //
    // Register 0x02 - DRCFG (Data Ready Config):
    //  The default value of this register is 0x81.
    //  POLARITY - DR pin Polarity Select (RW).
    //      1 = DR pin is active-high.
    //      0 = DR pin is active-low.
    //  EDGE - DR pin Edge/Level Select (RW).
    //      1 = DR pin asserts for 1 pulse.
    //      0 = DR pin asserts until STATUS is read.
    //  FORCE - Force DR pin to assert, this bit auto-clears (RW).
    //      1 = Force DR pin to assert.
    //      0 = normal DR operation.
    //  EN - Enable DR (RW).
    //      1 = DR enabled.
    //      0 = DR always negated.
    //
    private static final int DRCFG_POLARITY = (1 << 0);
    private static final int DRCFG_EDGE = (1 << 1);
    private static final int DRCFG_FORCE = (1 << 6);
    private static final int DRCFG_EN = (1 << 7);

    //
    // Register 0x04 - Last Detected Gesture (RO).
    //  The most recent gesture appears in this register. The gesture value remains until a new gesture is detected.
    //  The gesture bits in the status register can be used to determine when to read a new value from this register.
    //  0x01 - Right Swipe.
    //  0x02 - Left Swipe
    //  0x03 - Up Swipe
    //  0x05 - Hover
    //  0x06 - Hover-Left
    //  0x07 - Hover-Right
    //  0x08 - Hover-Up
    //
    private static final int GESTURE_NONE = 0x00;
    private static final int GESTURE_RIGHT_SWIPE = 0x01;
    private static final int GESTURE_LEFT_SWIPE = 0x02;
    private static final int GESTURE_UP_SWIPE = 0x03;
    private static final int GESTURE_HOVER = 0x05;
    private static final int GESTURE_HOVER_LEFT = 0x06;
    private static final int GESTURE_HOVER_RIGHT = 0x07;
    private static final int GESTURE_HOVER_UP = 0x08;

    /**
     * Specifies the various detected gestures.
     */
    public enum Gesture {
        NONE(GESTURE_NONE),
        RIGHT_SWIPE(GESTURE_RIGHT_SWIPE),
        LEFT_SWIPE(GESTURE_LEFT_SWIPE),
        UP_SWIPE(GESTURE_UP_SWIPE),
        HOVER(GESTURE_HOVER),
        HOVER_LEFT(GESTURE_HOVER_LEFT),
        HOVER_RIGHT(GESTURE_HOVER_RIGHT),
        HOVER_UP(GESTURE_HOVER_UP);

        public final int value;

        /**
         * Constructor: Create an instance of the enum type.
         *
         * @param value specifies the enum ordinal value.
         */
        private Gesture(int value) {
            this.value = value;
        }   //Gesture

        /**
         * This method returns the Gesture enum object matching the specified ordinal value.
         *
         * @param value specifies the ordinal value to match for.
         * @return Gesture enum object matching the ordinal value.
         */
        public static Gesture getGesture(int value) {
            for (Gesture g : Gesture.values()) {
                if (value == g.value) {
                    return g;
                }
            }
            return NONE;
        }   //getGesture

    }   //enum Gesture

    //
    // Register 0x05 - Last Detected Gesture Speed (RO).
    //  The speed of the most recently detected gesture is stored here. The value remains until a new gesture is
    //  detected.
    //

    //
    // Register 0x06 - Data Confidence Metric (RO).
    //  Currently unused. Returns 0.
    //

    //
    // Register 0x08 - X Position (RO).
    //  The most recently calculated X position is stored in this register.
    //
    private static final int MAX_XPOSITION = 240;

    //
    // Register 0x0a - Z Position (RO).
    //  The most recently calculated Z position is stored in this register.
    //
    private static final int MAX_ZPOSITION = 240;

    //
    // Register 0x0c - Left Emitter Ranging Data (RO).
    //  The left emitter ranging value is stored in this register.
    //

    //
    // Register 0x0e - Right Emitter Ranging Data (RO).
    //  The right emitter ranging value is stored in this register.
    //

    //
    // Register 0xfe - Register Map Version (RO).
    //  This register is used to identify the register map version of attached sensor. All sensors share a register
    //  map. Sensors with the same register map have the same value arrangement.
    //  0x01 = Register Map v1.
    //
    public static final int REGISTERMAP_VERSION = 0x01;

    //
    // Register 0xff - Sensor Model (RO).
    //  This register is used to identify the type of sensor attached.
    //  0x01 = XZ01.
    //
    public static final int MODEL_VERSION = 0x01;

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "ZX Distance";
    }

    public ZXDistanceSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                0, REG_RRNG + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(DEF_I2CADDRESS));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void setI2cAddress(I2cAddr i2cAddr) {
        deviceClient.setI2cAddress(i2cAddr);
    }

    public static int bytesToInt(byte low, byte high)
    {
        return ((int)low & 0xff) | (((int)high & 0xff) << 8);
    }   //bytesToInt

    public static int bytesToInt(byte data)
    {
        return bytesToInt(data, (byte)0);
    }   //bytesToInt

    public int getStatus() {
        byte[] data = deviceClient.read(REG_STATUS, 1);
        return bytesToInt(data[0]);
    }

    public int getDataReadyEnable() {
        byte[] data = deviceClient.read(REG_DRE, 1);
        return bytesToInt(data[0]);
    }

    public int getDataReadyConfiguration() {
        byte[] data = deviceClient.read(REG_DRCFG, 1);
        return bytesToInt(data[0]);
    }

    public int getGesture() {
        byte[] data = deviceClient.read(REG_GESTURE, 1);
        return bytesToInt(data[0]);
    }

    public int getGestureSpeed() {
        byte[] data = deviceClient.read(REG_GSPEED, 1);
        return bytesToInt(data[0]);
    }


    public int getDataConfigence() {
        byte[] data = deviceClient.read(REG_DCM, 1);
        return bytesToInt(data[0]);
    }

    public int getX() {
        byte[] data = deviceClient.read(REG_XPOS, 1);
        return bytesToInt(data[0]);
    }

    public int getZ() {
        byte[] data = deviceClient.read(REG_ZPOS, 1);
        return bytesToInt(data[0]);
    }

    public int getLeftRange() {
        byte[] data = deviceClient.read(REG_LRNG, 1);
        return bytesToInt(data[0]);
    }

    public int getRightRange() {
        byte[] data = deviceClient.read(REG_RRNG, 1);
        return bytesToInt(data[0]);
    }

    public int getRegisterMapVersion() {
        byte[] data = deviceClient.read(REG_REGVER, 1);
        return bytesToInt(data[0]);
    }

    public int getModel() {
        byte[] data = deviceClient.read(REG_MODEL, 1);
        return bytesToInt(data[0]);
    }

}
