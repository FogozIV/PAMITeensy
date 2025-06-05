print(">>> GLOBAL SCRIPT RAN <<<")
import json
from pathlib import Path
Import("env")

# Load config
config_path = Path(env["PROJECT_DIR"]) / "device_config.json"
with open(config_path) as f:
    config = json.load(f)

hostname = config.get("hostname", "defaultHostname")

# Set upload port dynamically
env.Replace(UPLOAD_PORT=f"{hostname}.local")
escaped = '\\"' + hostname + '\\"'  # becomes \"mainrobotTeensy\"
env.Append(CPPDEFINES=[f'HOSTNAME={escaped}'])