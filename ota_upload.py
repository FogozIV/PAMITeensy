import serial
import time
import os

from serial.serialutil import SerialException
from tqdm import tqdm
#from SCons.Script import DefaultEnvironment

#env = DefaultEnvironment()

message_header = "[Custom Upload] "

class UploadState:
    INIT = ("INIT", 1)
    SEND_FLASH_CMD = ("SEND_FLASH_CMD", 1)
    WAIT_FOR_GOOD_STATE_CONFIRMATION = ("WAIT_FOR_GOOD_STATE_CONFIRMATION", 2)
    INSIDE_FLASH = ("INSIDE_FLASH", 4)
    WAIT_CREATED_BUFFER = ("WAIT_CREATED_BUFFER", 100)
    WAIT_FOR_READY = ("WAIT_FOR_READY", 1)
    SEND_FIRMWARE = ("SEND_FIRMWARE", 100)
    VERIFICATION = ("VERIFICATION", 1)
    #not working yet
    INTERACTIVE = ("INTERACTIVE", 0)
    WAIT_FOR_REBOOT = ("WAIT_FOR_REBOOT", 0)
    FLASH_MOVE = ("FLASH_MOVE", 1)
    WAIT_DONE = ("WAIT_DONE", 20)
    DONE = ("DONE", 0)
class TeensyUploader:

    def __init__(self, serial : serial.Serial, firmware_path: str):
        self.previous_state = UploadState.INIT
        self.state = UploadState.INIT
        self.serial = serial
        self.filename = firmware_path
    def run(self):
        previous_time = time.time()
        while(not (self.state is UploadState.DONE)):
            if(self.previous_state != self.state):
                previous_time = time.time()
                self.previous_state = self.state
            if(self.state[1]!=0 and time.time() - previous_time > self.state[1]):
                raise Exception(message_header + "Waited too much time for state to end " + self.state[0])
            self.step()

    def wait_ready(self, timeout=2, error="Waited too much time for serial"):
        start = time.time()
        while(not self.serial.in_waiting and time.time() - start < timeout):
            pass
        if(not self.serial.in_waiting):
            raise Exception(message_header + error)

    def step(self):
        match self.state:
            case UploadState.INIT:
                self.state = UploadState.SEND_FLASH_CMD
            case UploadState.SEND_FLASH_CMD:
                self.serial.write(b'flash\n')
                self.serial.flush()
                self.state = UploadState.WAIT_FOR_GOOD_STATE_CONFIRMATION
                print(message_header + "Sending flash command")

            case UploadState.WAIT_FOR_GOOD_STATE_CONFIRMATION:
                self.wait_ready(8)
                line = self.serial.readline()
                if not line.startswith(b'flash'):
                    raise Exception(message_header + "Unexpected line received " + line.decode().strip())
                print(message_header + "Teensy was in a good state for flash")
                self.state = UploadState.INSIDE_FLASH
                if(b"unable" in line):
                    raise Exception(message_header + line.decode().strip())
                if(b'created' in line):
                    print(message_header + line.decode().strip())
                    self.state = UploadState.WAIT_FOR_READY
            case UploadState.INSIDE_FLASH:
                self.wait_ready(4)
                line = self.serial.readline()
                print(message_header + "Inside flash")
                if(b'Beginning the flash : ' in line):
                    self.state = UploadState.WAIT_CREATED_BUFFER

            case UploadState.WAIT_CREATED_BUFFER:
                self.wait_ready(timeout=30)
                print(message_header + "Waiting for buffer creation")
                line = self.serial.readline()
                if(b"unable" in line):
                    raise Exception(message_header + line.decode().strip())
                if(b'created' in line):
                    print(message_header + line.decode().strip())
                    self.state = UploadState.WAIT_FOR_READY
                elif(b'READY' in line):
                    print(message_header + "READY for upload")
                    self.state = UploadState.SEND_FIRMWARE
                else:
                    print(message_header + line.decode().strip())
            case UploadState.WAIT_FOR_READY:
                self.wait_ready()
                line = self.serial.readline()
                if(b"READY" in line):
                    print(message_header + "READY for upload")
                    self.state = UploadState.SEND_FIRMWARE
            case UploadState.SEND_FIRMWARE:
                print(message_header + "Sending firmware ...")
                self.serial.readlines()
                file_size = os.path.getsize(self.filename)
                with open(self.filename, "rb") as fw, tqdm(total=file_size, unit='B', unit_scale=True, desc="Uploading") as pbar:
                    for line in fw:
                        # Send line
                        self.serial.write(line)
                        self.serial.flush()

                        # Wait for device to echo or respond
                        if self.serial.in_waiting:
                            response = self.serial.readline()

                            # Check for abort
                            if b'abort' in response.lower():
                                raise Exception(message_header + response.decode(errors="replace").strip())

                            # Retry if 'bad line' response is received
                            while b'bad line' in response.lower():
                                print(message_header + "Bad line detected, resending...")
                                print(message_header + response.decode(errors="replace").strip())
                                self.serial.write(line)
                                self.serial.flush()
                                time.sleep(0.01)
                                if(not self.serial.in_waiting):
                                    break
                                response = self.serial.readline()
                        # Update progress bar
                        pbar.update(len(line))
                print(message_header + "Firmware sent !")
                self.state = UploadState.VERIFICATION
            case UploadState.INTERACTIVE:
                #confirmation part
                self.wait_ready()
                line = self.serial.readline()
                if(b'hex file:' in line):
                    print(message_header + line.decode().strip())
                if(line.startswith(b"abort")):
                    raise Exception(message_header + line.decode().strip())
                if(b'enter' in line and b'to flash' in line):
                    print(message_header + line.decode().strip())
                    data = input()
                    self.serial.write(data.encode())
                if(b'calling flash_move()' in line):
                    print(message_header + "Moving flash")
                    self.state = UploadState.DONE
            case UploadState.VERIFICATION:
                self.wait_ready()
                line = self.serial.readline()
                if(line.startswith(b"abort")):
                    raise Exception(message_header + line.decode().strip())
                if(b'hex file:' in line):
                    print(message_header + line.decode().strip())
                    self.state = UploadState.FLASH_MOVE
            case UploadState.FLASH_MOVE:
                self.wait_ready()
                line = self.serial.readline()
                if(b'calling flash_move()' in line):
                    print(message_header + "Moving flash")
                    self.state = UploadState.WAIT_DONE
            case UploadState.WAIT_DONE:
                while(self.serial.is_open):
                    if(self.serial.in_waiting):
                        line = self.serial.readline()
                        if(line.startswith(b"LOG=")):
                            break
                        print(self.serial.readline().decode().strip())
                    pass
                print(message_header + "Done flashing thanks you for using OTA")
                self.state = UploadState.DONE



def custom_upload(source, target, env):
    upload_port = env.get("UPLOAD_PORT")
    if(upload_port is None):
        env.AutodetectUploadPort()
    upload_port = env.get("UPLOAD_PORT")

    baud = env.GetProjectOption("custom_baud")
    parity = env.GetProjectOption("custom_parity")

    print("Custom baud rate:", baud)
    print("Custom parity:", parity)
    firmware_path = env.subst(env.get("BUILD_DIR") + "/" + env.get("PROGNAME") + ".hex")
    baudrate = baud or 1000000
    print(message_header + "Opening serial connection to", upload_port, " using baudrate : ", baudrate)
    try:
        if(upload_port is None):
            raise Exception(message_header + "No upload port")
        with serial.Serial(upload_port, baudrate, timeout=2, parity=parity) as ser:
            time.sleep(1)
            ser.write(b"\r\n")
            ser.write(b'0\r\n')
            ser.readlines()

            uploader = TeensyUploader(ser, firmware_path)
            time.sleep(1)  # Allow Teensy to reset or initialize
            uploader.run()
        time.sleep(2)
    except OSError as e:
        if("ClearCommError failed (OSError(22" in str(e.args)):
            print(message_header + "Done uploading")
        else:
            raise e

    except Exception as e:
        print(message_header + "Error:", e)
        raise e

from SCons.Script import Import
Import("env")
env.Replace(UPLOADCMD=custom_upload)