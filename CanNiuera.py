import time
import threading

# Try to import python-can, but provide helpful error if missing
try:
    import can
except ImportError:
    print("Error: 'python-can' library is required but not found.")
    print("Please install it using: pip install python-can")
    can = None


class CanNiuera:
    """
    NIUERA V2G module CAN interface (keeps CanPhoenix-like structure).

    Identifier (29-bit):
      PROTNO(9) + PTP(1) + DSTADDR(8) + SRCADDR(8) + GROUP(3)

    Payload:
      WRITE (SET): [0x03, 0x00, RegHi, RegLo, D0, D1, D2, D3]
      READ:        [0x10, 0x00, RegHi, RegLo, 0x00, 0x00, 0x00, 0x00]
      RESP:        [0x42, Err,  RegHi, RegLo, D0, D1, D2, D3]
    """

    # -------------------------------
    # Protocol constants
    # -------------------------------
    PROTNO_DEFAULT = 0x061  # 9-bit

    # PTP
    PTP_BROADCAST = 0
    PTP_P2P = 1

    # Addresses
    MONITOR_ADDR_DEFAULT = 0xF0
    DST_BROADCAST_ALL = 0xFF
    DST_BROADCAST_GROUP = 0xFE

    # "Byte00" function codes (your requested naming)
    WRITE_BYTE00 = 0x03
    READ_BYTE00 = 0x10
    RESP_BYTE00 = 0x42

    # Response status codes (byte1 in RESP)
    RESP_OK = 0xF0
    RESP_FAIL = 0xF2

    # Registers
    REG_POWER_ONOFF = 0x0030      # 0x00000000 startup, 0x00010000 shutdown
    REG_SET_DC_LINK_V = 0x0077    # u32 mV
    REG_SET_DC_CURRENT = 0x0079   # s32 mA
    REG_READ_DC_VI = 0x000F       # table says 0x000F (some examples show 0x000E)
    REG_READ_DC_VI_ALT = 0x000E   # accept both to be safe
    REG_READ_STATUS = 0x0040      # status bits (u32)

    def __init__(
        self,
        interface="socketcan",
        channel="can2",
        bitrate=125000,
        dst_addr=DST_BROADCAST_ALL,
        src_addr=MONITOR_ADDR_DEFAULT,
        group=0,
        ptp=PTP_BROADCAST,
        debug=False,
    ):
        if can is None:
            raise ImportError("python-can library not installed.")

        self.channel = channel
        self.bitrate = bitrate
        self.interface_type = interface
        self.bus = None
        self.is_connected = False

        # NIUERA addressing
        self.dst_addr = dst_addr & 0xFF
        self.src_addr = src_addr & 0xFF
        self.group = group & 0x07
        self.ptp = 1 if ptp else 0

        # Debug prints
        self.debug = bool(debug)

        # Internal state (same names style)
        self.evse_max_voltage = 0
        self.evse_min_voltage = 0
        self.evse_max_current = 0
        self.evse_min_current = 0
        self.evse_max_power = 0

        self.evse_present_voltage = 0.0
        self.evse_present_current = 0.0

        self.ev_max_voltage = 0
        self.ev_min_voltage = 0
        self.ev_max_current = 0
        self.ev_min_current = 0
        self.ev_max_power = 0

        self.started = False
        self.last_status_bits = 0
        self.last_work_mode = None

        self._stop_event = threading.Event()
        self._receive_thread = None

        # Heartbeat loop (same pattern)
        self._heartbeat_active = False
        self._heartbeat_thread = None

        # Try to open connection
        try:
            self.bus = can.Bus(interface=self.interface_type, channel=self.channel, bitrate=self.bitrate)
            self.is_connected = True
            print(f"Connected to CAN interface: {self.interface_type}/{self.channel} @ {self.bitrate}")
        except Exception as e:
            print(f"Failed to open CAN connection: {e}")

        if self.is_connected:
            self._start_receive_thread()

    def _start_receive_thread(self):
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def _receive_loop(self):
        while not self._stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self._process_frame(msg)
            except Exception as e:
                print(f"[NIUERA] Error in receive loop: {e}")
                time.sleep(0.1)

    @staticmethod
    def _u16_be(b0, b1):
        return (b0 << 8) | b1

    @staticmethod
    def _s16_be(b0, b1):
        v = (b0 << 8) | b1
        return v - 0x10000 if v & 0x8000 else v

    @staticmethod
    def _u32_to_be_bytes(val: int):
        return list(int(val).to_bytes(4, byteorder="big", signed=False))

    @staticmethod
    def _i32_to_be_bytes(val: int):
        return list(int(val).to_bytes(4, byteorder="big", signed=True))

    def _build_identifier(self, dst_addr=None, src_addr=None, group=None, ptp=None, protno=None):
        protno = self.PROTNO_DEFAULT if protno is None else (protno & 0x1FF)
        ptp = self.ptp if ptp is None else (1 if ptp else 0)
        dst = self.dst_addr if dst_addr is None else (dst_addr & 0xFF)
        src = self.src_addr if src_addr is None else (src_addr & 0xFF)
        grp = self.group if group is None else (group & 0x07)

        ident = 0
        ident |= (protno & 0x1FF) << 20
        ident |= (ptp & 0x01) << 19
        ident |= (dst & 0xFF) << 11
        ident |= (src & 0xFF) << 3
        ident |= (grp & 0x07)
        return ident

    def _parse_identifier(self, arbitration_id: int):
        protno = (arbitration_id >> 20) & 0x1FF
        ptp = (arbitration_id >> 19) & 0x01
        dst = (arbitration_id >> 11) & 0xFF
        src = (arbitration_id >> 3) & 0xFF
        grp = arbitration_id & 0x07
        return protno, ptp, dst, src, grp

    def _send(self, data, dst_addr=None):
        if not self.is_connected:
            return False
        if len(data) != 8:
            raise ValueError("NIUERA frames must be exactly 8 bytes")

        ident = self._build_identifier(dst_addr=dst_addr)

        if self.debug:
            print(f"[TX] ID=0x{ident:08X} DATA={' '.join(f'{b:02X}' for b in data)}")

        try:
            msg = can.Message(arbitration_id=ident, data=data, is_extended_id=True)
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"[NIUERA] Error sending CAN frame: {e}")
            return False

    def _send_write_reg_u32(self, reg: int, value_u32: int, dst_addr=None):
        vb = self._u32_to_be_bytes(value_u32)
        data = [self.WRITE_BYTE00, 0x00, (reg >> 8) & 0xFF, reg & 0xFF] + vb
        return self._send(data, dst_addr=dst_addr)

    def _send_write_reg_i32(self, reg: int, value_i32: int, dst_addr=None):
        vb = self._i32_to_be_bytes(value_i32)
        data = [self.WRITE_BYTE00, 0x00, (reg >> 8) & 0xFF, reg & 0xFF] + vb
        return self._send(data, dst_addr=dst_addr)

    def _send_read_reg(self, reg: int, dst_addr=None):
        data = [self.READ_BYTE00, 0x00, (reg >> 8) & 0xFF, reg & 0xFF, 0, 0, 0, 0]
        return self._send(data, dst_addr=dst_addr)

    def _process_frame(self, msg):
        if not msg.is_extended_id:
            return

        protno, ptp, dst, src, grp = self._parse_identifier(msg.arbitration_id)
        if protno != self.PROTNO_DEFAULT:
            return

        payload = list(msg.data)
        if len(payload) < 8:
            return

        if self.debug:
            print(f"[RX] ID=0x{msg.arbitration_id:08X} DATA={' '.join(f'{b:02X}' for b in payload)}")

        fc = payload[0]

        # Response frame
        if fc == self.RESP_BYTE00:
            err = payload[1]
            reg = (payload[2] << 8) | payload[3]
            d4, d5, d6, d7 = payload[4], payload[5], payload[6], payload[7]

            if err != self.RESP_OK:
                return

            # DC V/I (accept 0x000F and 0x000E)
            if reg in (self.REG_READ_DC_VI, self.REG_READ_DC_VI_ALT):
                v_raw = self._u16_be(d4, d5)     # 0.1V
                i_raw = self._s16_be(d6, d7)     # 0.01A signed
                self.evse_present_voltage = v_raw / 10.0
                self.evse_present_current = i_raw / 100.0

            elif reg == self.REG_READ_STATUS:
                self.last_status_bits = (d4 << 24) | (d5 << 16) | (d6 << 8) | d7
                self.last_work_mode = (self.last_status_bits >> 12) & 0x03

    def start(self):
        """Power ON (NIUERA reg 0x0030 = 0x00000000)."""
        WRITE_BYTE00 = self.WRITE_BYTE00
        data = [WRITE_BYTE00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00]
        ok = self._send(data)
        if ok:
            self.started = True
        return ok

    def stop(self):
        """Power OFF (NIUERA reg 0x0030 = 0x00010000)."""
        WRITE_BYTE00 = self.WRITE_BYTE00
        data = [WRITE_BYTE00, 0x00, 0x00, 0x30, 0x00, 0x01, 0x00, 0x00]
        ok = self._send(data)
        if ok:
            self.started = False

        # Best effort safety
        try:
            self.setEvTargetCurrent(0)
            self.setEvTargetVoltage(0)
        except Exception:
            pass

        return ok

    def close(self):
        """Stops threads and closes bus."""
        self.StopCanLoop()

        self._stop_event.set()
        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)

        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass

        self.is_connected = False

# --- Setters for EVSE Capabilities ---
    def setEvseMaxCurrent(self, value):
        self.evse_max_current = value

    def setEvseMinCurrent(self, value):
        self.evse_min_current = value

    def setEvseMaxVoltage(self, value):
        self.evse_max_voltage = value

    def setEvseMinVoltage(self, value):
        self.evse_min_voltage = value

    def setEvseMaxPower(self, value):
        self.evse_max_power = value

    def setEvseDeltaVoltage(self, value):
        pass

    def setEvseDeltaCurrent(self, value):
        pass

    # --- Setters for EV Limits ---
    def setEvMaxCurrent(self, value):
        self.ev_max_current = value

    def setEvMinCurrent(self, value):
        self.ev_min_current = value

    def setEvMaxVoltage(self, value):
        self.ev_max_voltage = value

    def setEvMinVoltage(self, value):
        self.ev_min_voltage = value

    def setEvMinPower(self, value):
        pass

    def setEvMaxPower(self, value):
        self.ev_max_power = value

    # --- Dynamic Targets ---
    def setEvTargetVoltage(self, voltage):
        """NIUERA: reg 0x0077, value=mV (u32). Input volts."""
        if self.isVoltageLimitExceeded(voltage):
            return False

        WRITE_BYTE00 = self.WRITE_BYTE00
        voltage_mv = int(float(voltage) * 1000.0)
        val_bytes = voltage_mv.to_bytes(4, byteorder="big", signed=False)
        data = [WRITE_BYTE00, 0x00, 0x00, 0x77, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]]
        return self._send(data)

    def setEvTargetCurrent(self, current):
        """NIUERA: reg 0x0079, value=mA (s32). Input amps."""
        if self.isCurrentLimitExceeded(current):
            return False

        WRITE_BYTE00 = self.WRITE_BYTE00
        current_ma = int(float(current) * 1000.0)
        val_bytes = current_ma.to_bytes(4, byteorder="big", signed=True)
        data = [WRITE_BYTE00, 0x00, 0x00, 0x79, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]]
        return self._send(data)

    # --- Getters ---
    def getEvseMaxCurrent(self): return self.evse_max_current
    def getEvseMinCurrent(self): return self.evse_min_current
    def getEvseMaxVoltage(self): return self.evse_max_voltage
    def getEvseMinVoltage(self): return self.evse_min_voltage
    def getEvseMaxPower(self): return self.evse_max_power
    def getEvseDeltaVoltage(self): return 0
    def getEvseDeltaCurrent(self): return 0
    def getEvMaxCurrent(self): return self.ev_max_current
    def getEvMinCurrent(self): return self.ev_min_current
    def getEvMaxVoltage(self): return self.ev_max_voltage
    def getEvMinVoltage(self): return self.ev_min_voltage
    def getEvMinPower(self): return 0
    def getEvMaxPower(self): return self.ev_max_power

    # --- Real-time Values (Phoenix-like: build data locally) ---
    def getEvsePresentVoltage(self):
        """Request DC V/I and return cached voltage (V)."""
        READ_BYTE00 = self.READ_BYTE00
        data = [READ_BYTE00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00]
        self._send(data)
        return self.evse_present_voltage

    def getEvsePresentCurrent(self):
        """Request DC V/I and return cached current (A)."""
        READ_BYTE00 = self.READ_BYTE00
        data = [READ_BYTE00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00]
        self._send(data)
        return self.evse_present_current

    def getModuleStatusBits(self):
        """Request status bits and return cached bits."""
        READ_BYTE00 = self.READ_BYTE00
        data = [READ_BYTE00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00]
        self._send(data)
        return self.last_status_bits

    # --- Safety checks ---
    def isVoltageLimitExceeded(self, voltage):
        return voltage > self.evse_max_voltage or (self.evse_min_voltage > 0 and voltage < self.evse_min_voltage)

    def isCurrentLimitExceeded(self, current):
        return current > self.evse_max_current or (self.evse_min_current > 0 and current < self.evse_min_current)

    def isPowerLimitExceeded(self, power):
        return power > self.evse_max_power


    def StartCanLoop(self, period_s=5.0, keep_alive_read=True):
        """Keep CAN traffic alive (read DC V/I periodically)."""
        if not self._heartbeat_active:
            self._heartbeat_active = True
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_worker, args=(period_s, keep_alive_read), daemon=True
            )
            self._heartbeat_thread.start()
            print("[CAN] NIUERA Heartbeat Loop Started")

    def StopCanLoop(self):
        self._heartbeat_active = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=1.0)
        self._heartbeat_thread = None

    def _heartbeat_worker(self, period_s, keep_alive_read):
        while self._heartbeat_active:
            try:
                if self.started and keep_alive_read:
                    self._send_read_reg(self.REG_READ_DC_VI)
            except Exception as e:
                print(f"[NIUERA HEARTBEAT ERROR] {e}")
            time.sleep(float(period_s))

if __name__ == "__main__":
    print("Initializing CanNiuera...")
    try:
        charger = CanNiuera(interface="socketcan", channel="can2", bitrate=125000)
        charger.start()
        time.sleep(5)
        charger.stop()
        charger.close()
        print("Test Complete.")
    except ImportError:
        print("Skipping test: python-can not installed.")
    except Exception as e:
        print(f"Test failed (expected if vcan0 not up): {e}")

    '''
    try:
        charger = CanNiuera(interface="socketcan", channel="can2", bitrate=125000, debug=False)

        # Make sure your limit check won't reject these
        charger.setEvseMaxVoltage(600)   # must be >= 500
        charger.setEvseMinVoltage(0)

        charger.start()
        time.sleep(1.0)

        hold_s = 5.0
        read_interval_s = 0.2

        # Forward then reverse sequence (include 500 twice to hold it on the way down too)
        forward = [100, 200, 300, 400, 500]
        reverse = [500, 400, 300, 200, 100]
        sequence = forward + reverse

        print("[TEST] Sequence: 100->200->300->400->500 (5s each) then 500->400->300->200->100 (5s each)")

        for target_v in sequence:
            ok = charger.setEvTargetVoltage(float(target_v))
            if not ok:
                print(f"[ERROR] setEvTargetVoltage({target_v}) rejected by limit check.")
                break

            print(f"\n=== SET TARGET {target_v:.1f}V (hold {hold_s:.1f}s) ===")
            t0 = time.time()
            while (time.time() - t0) < hold_s:
                meas = charger.getEvsePresentVoltage()
                print(f"       Target={target_v:6.1f}V  Measured={meas:6.1f}V")
                time.sleep(read_interval_s)

        charger.stop()
        charger.close()
        print("Test Complete.")

    except ImportError:
        print("Skipping test: python-can not installed.")
    except Exception as e:
        print(f"Test failed: {e}")
    '''


    
