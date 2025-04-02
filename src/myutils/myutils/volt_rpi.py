#!/usr/bin/env python3
# 3/15/2025 measure and report internal voltage or rpi
import rclpy
import subprocess
import argparse

def main():
    rclpy.init()
    
def measure_voltage(domain="core"):
    try:
        # Run vcgencmd to measure voltage
        output = subprocess.check_output(["vcgencmd", "measure_volts", domain], text=True)
        voltage_str = output.strip().split("=")[1].replace("V", "")
        voltage = float(voltage_str)
        return f"{voltage:.4f} V"
    except subprocess.CalledProcessError as e:
        return f"Error: {e}"
    except Exception as e:
        return f"Unexpected error: {e}"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Measure Raspberry Pi voltage.")
    parser.add_argument("--domain", type=str, default="core", help="Voltage domain (e.g., core, sdram_c, sdram_i, sdram_p)")
    args = parser.parse_args()
   
    result = measure_voltage(args.domain)
    print(f"{args.domain} voltage: {result}")
    