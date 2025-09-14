from GUI import *
from  Comm import *
try:
    init_serial()
    threading.Thread(target=select_state, daemon=True).start()
    # ---------------- Start GUI ----------------
    init_GUI()
    # ---------------- Cleanup ----------------
    stop_state.set()

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    if 'ser' in locals() and ser is not None and ser.is_open:
        ser.close()

