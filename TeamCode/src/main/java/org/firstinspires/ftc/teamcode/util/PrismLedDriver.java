package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

/**
 * goBILDA Prism RGB LED Driver (I2C 0x38)
 *
 * Supports:
 *  - Set LED count
 *  - Clear animations
 *  - Write full custom RGB frame (24 LEDs = 72 bytes)
 */
public class PrismLedDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final int I2C_ADDR_7BIT = 0x38;

    // Registers
    private static final int REG_CUSTOM_LEDS = 0x00;
    private static final int REG_STATUS      = 0x05;
    private static final int REG_CONTROL     = 0x06;

    public enum PixelOrder { RGB, GRB }

    public PrismLedDriver(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDR_7BIT));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public static PrismLedDriver fromHardwareMap(HardwareMap hm, String name) {
        I2cDeviceSynch dev = hm.get(I2cDeviceSynch.class, name);
        return new PrismLedDriver(dev);
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    // -------------------- Required HardwareDevice Overrides --------------------

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA Prism RGB LED Driver";
    }

    @Override
    public String getConnectionInfo() {
        return deviceClient.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        // Nothing special required
    }

    @Override
    public void close() {
        super.close();
    }

    // -------------------- Public API --------------------

    public void clearAnimations() {
        writeControl32(1 << 25); // bit 25 = clear animations
        sleepQuiet(10);
    }

    public boolean setLedCountVerified(int ledCount) {
        ledCount = clamp(ledCount, 0, 255);

        int controlWord = ((ledCount & 0xFF) << 16) | (1 << 24);

        write32(REG_CONTROL, controlWord, Endian.LITTLE);
        sleepQuiet(10);

        int status = read32(REG_STATUS, Endian.LITTLE);
        int statusLedCount = status & 0xFF;
        return statusLedCount == ledCount;
    }

    public void writeCustomFrameRgb(byte[] frameRgb, PixelOrder order) {
        if (frameRgb == null) return;

        byte[] out = frameRgb;

        if (order == PixelOrder.GRB) {
            out = new byte[frameRgb.length];
            for (int i = 0; i < frameRgb.length; i += 3) {
                byte r = frameRgb[i];
                byte g = frameRgb[i + 1];
                byte b = frameRgb[i + 2];

                out[i]     = g;
                out[i + 1] = r;
                out[i + 2] = b;
            }
        }

        deviceClient.write(REG_CUSTOM_LEDS, out);

        // Apply update (bit 24)
        writeControl32(1 << 24);
    }

    // -------------------- Low-level helpers --------------------

    private enum Endian { LITTLE, BIG }

    private void writeControl32(int controlWord) {
        write32(REG_CONTROL, controlWord, Endian.LITTLE);
    }

    private void write32(int reg, int value, Endian endian) {
        byte[] b = new byte[4];

        if (endian == Endian.LITTLE) {
            b[0] = (byte) (value & 0xFF);
            b[1] = (byte) ((value >> 8) & 0xFF);
            b[2] = (byte) ((value >> 16) & 0xFF);
            b[3] = (byte) ((value >> 24) & 0xFF);
        } else {
            b[3] = (byte) (value & 0xFF);
            b[2] = (byte) ((value >> 8) & 0xFF);
            b[1] = (byte) ((value >> 16) & 0xFF);
            b[0] = (byte) ((value >> 24) & 0xFF);
        }

        deviceClient.write(reg, b);
    }

    private int read32(int reg, Endian endian) {
        byte[] b = deviceClient.read(reg, 4);
        if (b == null || b.length < 4) return 0;

        if (endian == Endian.LITTLE) {
            return (b[0] & 0xFF)
                    | ((b[1] & 0xFF) << 8)
                    | ((b[2] & 0xFF) << 16)
                    | ((b[3] & 0xFF) << 24);
        } else {
            return (b[3] & 0xFF)
                    | ((b[2] & 0xFF) << 8)
                    | ((b[1] & 0xFF) << 16)
                    | ((b[0] & 0xFF) << 24);
        }
    }

    private static int clamp(int x, int lo, int hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static void sleepQuiet(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }
}
