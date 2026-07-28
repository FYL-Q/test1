import os
print("\n===== 迅速科技两轮车_上位机123 =====\n")  # 优先打印你的信息
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"  # 禁用欢迎信息
import pygame
import serial
import serial.tools.list_ports
import struct
import numpy
import sys
import msvcrt  # Windows 专用（任意键退出）
import time


# 打包成一个exe文件——pyinstaller -F 1.py

# Global variables
obj = numpy.zeros((2, 100), dtype=float)  # 2x100的浮点数矩阵
ptobj = 0  # 当前帧中检测到的对象数量
zhen = 0  # 帧号 (将由 RadarDataParser 更新)
oldzhen = 0  # Not used currently
alarmStatus = 0  # 状态


class RadarDataParser:
    def __init__(self, port: str = None, baudrate: int = 256000):
        self.ser = self._open_serial_port(port, baudrate)
        self.buffer = bytearray()
        self.obj_internal = numpy.zeros((2, 100), dtype=float)  # Use a separate internal array
        self.current_frame_length = 0
        self.HEADER = bytes([0xAA, 0x55])

        # New attributes to hold the latest parsed data for external access
        self.latest_parsed_obj = numpy.zeros((2, 100), dtype=float)
        self.latest_parsed_ptobj = 0
        self.parsed_frame_count = 0  # To track total frames parsed by this instance

    def _open_serial_port(self, port: str, baudrate: int) -> serial.Serial:
        try:
            def list_available_ports():
                """列出当前所有可用串口"""
                ports = serial.tools.list_ports.comports()
                if not ports:
                    print("\n警告：当前没有可用的串口设备！")
                    print("请插入设备后输入 'r/R' 并回车以刷新COM设备列表，或直接按回车键退出。")
                    return []

                print("\n当前可用的串口设备：")
                for i, p in enumerate(ports, 1):
                    # 提取纯描述信息（去掉可能的重复 COM 口）
                    description = p.description
                    if p.device in description:
                        # 如果描述信息包含 COM 口（如 "USB-SERIAL CH340 (COM9)"），去掉括号部分
                        description = description.split("(")[0].strip()

                    # 合并 COM 口和描述信息
                    port_info = f"{p.device}"
                    if description:
                        port_info += f" ({description})"

                    print(f"  {i}. {port_info}")

                return ports

            while True:
                ports = list_available_ports()

                # 如果没有串口，等待用户操作
                if not ports:
                    user_input = input().strip().lower()
                    if user_input == 'r':
                        continue  # 重新检测
                    else:
                        print("程序退出。")
                        sys.exit(0)

                # 有串口时让用户选择
                while True:
                    try:
                        user_input = input("\n请输入要使用的串口编号（1-%d），或输入 'r/R' 并回车以刷新COM设备列表: " % len(ports))

                        if user_input.lower() == 'r':
                            break  # 跳出内层循环，重新检测串口

                        choice = int(user_input)
                        if 1 <= choice <= len(ports):
                            selected_port = ports[choice - 1].device
                            print(f"已选择: {selected_port}")
                            return serial.Serial(selected_port, baudrate, timeout=0.05)
                        else:
                            print("输入的编号超出范围！")
                    except ValueError:
                        print("请输入有效的数字或 'r/R' 并回车以刷新COM设备列表！")
                # 如果选择刷新，会通过break跳到这里，外层循环会重新执行

        except Exception as e:
            print(f"串口初始化失败: {e}")
            print("请按回车键退出程序...")
            input()
            sys.exit(1)

    def calculate_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def find_frame(self) -> tuple:
        header_pos = self.buffer.find(self.HEADER)
        if header_pos == -1:
            return False, 0
        self.buffer = self.buffer[header_pos:]
        if len(self.buffer) < 3:
            return False, 0
        num_targets = self.buffer[2]
        # Basic sanity check for num_targets to prevent oversized frame_length
        if num_targets > 8 or num_targets < 0:  # Assuming max 8 targets as per old code
            self.buffer = self.buffer[1:]  # Discard current byte, try next
            return False, 0

        # Calculate expected frame length
        # Header (2) + num_targets (1) + Reserved (7) + Targets Data (num_targets * 5) + CRC (2)
        self.current_frame_length = 2 + 1 + 7 + num_targets * 5 + 2
        return len(self.buffer) >= self.current_frame_length, header_pos

    def parse_frame(self) -> bool:
        has_frame, _ = self.find_frame()
        if not has_frame or len(self.buffer) < self.current_frame_length:
            return False

        frame = self.buffer[:self.current_frame_length]

        # CRC check
        if self.calculate_crc(frame[:-2]) != int.from_bytes(frame[-2:], 'little'):
            # print(f"CRC Mismatch! Expected: {int.from_bytes(frame[-2:], 'little'):04X}, Calculated: {self.calculate_crc(frame[:-2]):04X}")
            self.buffer = self.buffer[1:]  # Discard first byte and try to find next header
            return False

        try:
            pos = 3  # Skip Header (2) + Target Count (1)
            num_targets = frame[2]

            # Clear previous target data in the internal array for current frame
            self.obj_internal.fill(0)  # Clear previous data before filling new

            pos += 7  # Skip Reserved bytes

            for i in range(num_targets):
                if pos + 5 > len(frame):  # Check if enough bytes remain for this target
                    # This indicates truncated frame data, might happen with corrupted packets
                    # In this case, discard the whole frame and return False
                    self.buffer = self.buffer[self.current_frame_length:]
                    return False

                target_id = frame[pos]
                # Use struct.unpack for more robust byte conversion
                # '>H' for big-endian unsigned short (2 bytes)
                # '>h' for big-endian signed short (2 bytes) - assuming speed could be negative
                distance = struct.unpack('>H', frame[pos + 1:pos + 3])[0] / 100.0  # Unsigned short for distance——m
                speed = struct.unpack('>h', frame[pos + 3:pos + 5])[
                            0] / 100.0  # Signed short for speed (if speed can be negative)——Km/h

                self.obj_internal[0, i] = distance
                self.obj_internal[1, i] = speed
                pos += 5

            # Successfully parsed a frame!
            # Now, copy the internal data to the external-facing attributes
            self.latest_parsed_obj[:] = self.obj_internal  # Use slicing to update in-place for efficiency
            self.latest_parsed_ptobj = num_targets
            self.parsed_frame_count += 1  # Increment frame counter

            self.buffer = self.buffer[self.current_frame_length:]
            return True  # Indicate successful parsing of one frame

        except Exception as e:
            # print(f"Error during frame parsing: {e}, Frame data: {' '.join(f'{b:02X}' for b in frame)}")
            self.buffer = self.buffer[self.current_frame_length:]  # Discard bad frame
            return False

    # Modified read_data: it should only read available data once per call
    # and process all complete frames it finds. It should NOT loop indefinitely.
    def read_data(self) -> bool:
        new_frame_processed = False
        if self.ser.in_waiting > 0:
            bytes_read = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(bytes_read)
            # print(f"Read {len(bytes_read)} bytes, buffer len: {len(self.buffer)}") # Debugging

        # Process all complete frames in the buffer
        while self.parse_frame():
            new_frame_processed = True
            # print("Parsed a frame.") # Debugging

        # If buffer grows too large without a valid header, trim it
        # This prevents infinite growth from corrupted data streams
        if not self.find_frame()[0] and len(self.buffer) > 200:  # Increased threshold for trimming
            self.buffer = self.buffer[-100:]  # Keep last 100 bytes
            # print("Buffer trimmed due to no frame found and large size.") # Debugging

        return new_frame_processed  # Return True if at least one new frame was processed

    def close(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")


def drawWin(screen, font):
    global zhen, obj, ptobj, alarmStatus  # Declare globals used
    screen.fill("white")

    # Draw distance grid lines and labels
    for i in range(0, 9):  # 0m, 20m, ..., 160m
        text_line = font.render(str(i * 20) + "m", True, (0, 0, 255))
        screen.blit(text_line, (215, int(i * 20 * 4)))

    # Display current frame time
    textTime = font.render(format(zhen * 0.05, '.2f'), True, (255, 0, 0))
    screen.blit(textTime, (10, 10))

    # Draw detected targets
    for i in range(0, ptobj):  # Iterate through all detected targets
        # Safety check for division by zero (if speed is 0)
        if obj[1, i] == 0:
            t = float('inf')  # Set TTC to infinity if speed is zero
        else:
            speed_mps = obj[1, i] / 3.6
            t = obj[0, i] / speed_mps  # Calculate TTC = Distance / Speed

        # Ensure object distance is positive and within reasonable screen limits
        if obj[0, i] < 0: continue  # Don't draw negative distances (behind sensor)

        y_pos = int(obj[0, i] * 4)  # Convert meters to pixels
        if y_pos > 680: continue  # Don't draw if too far off screen

        rect = pygame.Rect((130, y_pos), (20, 30))  # Target rectangle

        # Color targets based on risk/TTC
        # Red: High speed approaching (approx. >30km/h or 8.3m/s) AND TTC 0-15s
        if obj[1, i] > 30 and 0.0 < t < 15.0:
            pygame.draw.rect(screen, 'red', rect)
            timetext = font.render(f"{t:.1f}s | {obj[1, i]:.1f}km/h", True, (255, 0, 0))
            screen.blit(timetext, (115, y_pos - 15))
        # Blue: TTC 0-15s (but not necessarily high speed approaching)
        elif 0.0 < t < 15.0:
            pygame.draw.rect(screen, 'blue', rect)
            timetext = font.render(f"{t:.1f}s | {obj[1, i]:.1f}km/h", True, (0, 0, 255))
            screen.blit(timetext, (115, y_pos - 15))
        else:  # Green: Other cases (TTC too long, invalid, or not approaching)
            pygame.draw.rect(screen, 'green', rect)
            timetext = font.render(f"{t:.1f}s | {obj[1, i]:.1f}km/h", True, (0, 128, 0))  # Darker green for text
            screen.blit(timetext, (115, y_pos - 15))

    # Draw alarm status indicator
    if alarmStatus == 1:
        pygame.draw.circle(screen, "blue", (20, 95), 15, 0)
    elif alarmStatus == 2:
        pygame.draw.circle(screen, "red", (20, 95), 15, 0)

    pygame.display.flip()


def theHost():
    parser = None  # Initialize parser to None
    try:
        parser = RadarDataParser()
        print("Open serial port success!")
    except RuntimeError as e:  # Catch specific RuntimeError from _open_serial_port
        print(f"Error opening serial port: {e}")
        exit(0)
    except Exception as e:  # Catch any other unexpected errors during parser initialization
        print(f"An unexpected error occurred during parser initialization: {e}")
        exit(0)

    pygame.init()

    screen = pygame.display.set_mode((250, 700))
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.SysFont('', 20)

    screen.fill("white")
    pygame.display.flip()

    while running:
        # 1. Handle Pygame events (important for responsiveness and closing the window)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 2. Read and parse data from the serial port
        # read_data returns True if at least one new frame was successfully processed
        if parser.read_data():
            # 3. Synchronize global variables with the latest parsed data from the parser
            global obj, ptobj, zhen
            obj[:] = parser.latest_parsed_obj  # Update global obj in-place
            ptobj = parser.latest_parsed_ptobj  # Update global ptobj
            zhen = parser.parsed_frame_count  # Update global zhen

            # 4. Draw the visualization only if new data arrived
            drawWin(screen, font)

        # 5. Control the frame rate to avoid consuming 100% CPU
        clock.tick(30)  # Limit to 30 frames per second

    # Clean up resources when the main loop ends
    if parser:
        parser.close()
    pygame.quit()


if __name__ == '__main__':
    theHost()

