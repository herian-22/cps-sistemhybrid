import datetime
import os

CACHE_DIR = "cache"
LOG_FILE = os.path.join(CACHE_DIR, "session_log.txt")

def log(message, level="INFO"):
    """Logs a message to the session log file with a timestamp."""
    if not os.path.exists(CACHE_DIR):
        os.makedirs(CACHE_DIR)
        
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    log_entry = f"[{timestamp}] [{level}] {message}\n"
    
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(log_entry)
        # Also print to terminal for convenience
        print(log_entry.strip())
    except Exception as e:
        print(f"Failed to write to log file: {e}")

def clear_logs():
    """Clears the log file."""
    if os.path.exists(LOG_FILE):
        os.remove(LOG_FILE)
