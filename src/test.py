import serial
import time

def reset_esp32_gnss(ser):
    """Reset ESP32 with u-blox GNSS via Serial2"""
    try:
        # Method 1: Use DTR to reset ESP32 (triggers bootloader)
        ser.setDTR(False)
        ser.setDTR(True)
        
        # Clear any boot messages
        # ser.reset_input_buffer()
        
        # Read and discard boot messages
        while True:
            boot_output = ser.readline().decode('utf-8', errors='ignore')
            print(f"Boot: {boot_output.strip()}")
            if "Serial2 started" in boot_output:
                break
                
        return True
        
    except Exception as e:
        print(f"Reset failed: {e}")
        return False

# Usage
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
reset_esp32_gnss(ser)