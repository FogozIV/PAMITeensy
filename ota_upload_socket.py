

import serial
import time
import os
import subprocess

from SCons.Script import Import
Import("env")
def parse_upload_flags(upload_flags):
    parsed = {}
    for flag in upload_flags:
        if flag.startswith("-") and "=" in flag:
            key, val = flag.lstrip("-").split("=", 1)
            parsed[key] = val
    return parsed
def custom_upload(source, target, env):
    firmware_path = env.subst(env.get("BUILD_DIR") + "/" + env.get("PROGNAME") + ".hex")
    ip = env.get("UPLOAD_PORT")
    flags = parse_upload_flags(env.get("UPLOAD_FLAGS", []))
    port = flags.get("host_port")
    print(f"Port : {port} IP : {ip}, firmware_path : {firmware_path}")
    subprocess.run(["TeensyUploader", ip, port, firmware_path])

env.Replace(UPLOADCMD=custom_upload)