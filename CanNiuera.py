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
    NIUERA Charging Module (X-EV series) CAN interface, based on:
    - "V2G Module Communication Protocol V1.3" (Baud rate 125 kbps)
      Identifier: PROTNO(9) + PTP(1) + DSTADDR(8) + SRCADDR(8) + Group(3)
      Payload:
        Set:  03 00 [RegHi RegLo] [Data(4 bytes)]
        Read: 10 00 [RegHi RegLo] 00 00 00 00
        Resp: 42 [Err] [RegHi RegLo] [Data(4 bytes)]
    Keeps the same structure/pattern as your CanPhoenix class (start/stop, getters/setters, background RX loop).
    """

    # --- CAN Protocol constants (NIUERA) ---
    PROTNO_DEFAULT = 0x061  # 9-bit protocol number :contentReference[oaicite:1]{index=1}

    # PTP
    PTP_BROADCAST = 0
    PTP_P2P = 1

    # Addresses
    मॉODULE_ADDR_MIN = 0x00
    MODULE_ADDR_MAX = 0x3C  # (some docs say 00~60 decimal; panel default 0x00) :contentReference[oaicite:2]{index=2}
    MONITOR_ADDR_DEFAULT = 0xF0  # monitoring address fixed :contentReference[oaicite:3]{index=3}
    DST_BROADCAST_ALL = 0xFF      # broadcast address :contentReference[oaicite:4]{index=4}
    DST_BROADCAST_GROUP = 0xFE    # intra-group broadcast :contentReference[oaicite:5]{index=5}

    # Function codes
    FC_SET = 0x03
    FC_READ = 0x10
    FC_RESP_INT = 0x42

    # Response error codes (byte1)
    RESP_OK = 0xF0
    RESP_FAIL = 0xF2

    # Registers we need (Table 1)
    REG_READ_DC_VI = 0x000F      # Read DC voltage/current (0.1V, 0.01A signed) :contentReference[oaicite:6]{index=6}
    REG_POWER_ONOFF = 0x0030     # 0x00000000 startup, 0x00010000 shutdown :contentReference[oaicite:7]{index=7}
    REG_SET_DC_LINK_V = 0x0077   # mV :contentReference[oaicite:8]{index=8}
    REG_SET_DC_CURRENT = 0x0079  # mA (signed: +rectifier, -inverter) :contentReference[oaicite:9]{index=9}
    REG_READ_STATUS = 0x0040     # alarm/status bits :contentReference[oaicite:10]{index=10}
    REG_SET_WORKMODE = 0x002F    # 0 grid, 1 off-grid, 2 rectifier (only when powered off) :contentReference[oaicite:11]{index=11}

    # Work modes (optional helpers)
    MODE_GRID = 0
    MODE_OFFGRID = 1
    MODE_RECTIFIER = 2

    def __init__(
        self,
        interface="socketcan",
        channel="can2",
        bitrate=125000,
        dst_addr=DST_BROADCAST_ALL,
        src_addr=MONITOR_ADDR_DEFAULT,
        group=0,
        ptp=PTP_BROADCAST,
    ):
        if can is None:
            raise ImportError("python-can library not installed.")

        self.channel = channel
        self.bitrate = bitrate
        self.interface_type = interface
        self.bus = None
        self.is_connected = False

        # NIUERA addressing
        self.dst_addr = dst_addr
        self.src_addr = src_addr
        self.group = group & 0x07
        self.ptp = 1 if ptp else 0

        # Internal state (keep same names as CanPhoenix pattern)
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

        try:
            # For socketcan, bitrate is usually set at OS level; we pass anyway for consistency.
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
    def _i32_to_be_bytes(val: int):
        return list(int(val).to_bytes(4, byteorder="big", signed=True))

    @staticmethod
    def _u32_to_be_bytes(val: int):
        return list(int(val).to_bytes(4, byteorder="big", signed=False))

    def _build_identifier(self, dst_addr=None, src_addr=None, group=None, ptp=None, protno=None):
        """
        29-bit identifier:
          bits 28..20: PROTNO (9 bits)
          bit  19    : PTP (1 bit)
          bits 18..11: DSTADDR (8 bits)
          bits 10..3 : SRCADDR (8 bits)
          bits 2..0  : Group (3 bits)
        :contentReference[oaicite:12]{index=12}
        """
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
            raise ValueError("NIUERA frames must be 8 bytes data.")

        ident = self._build_identifier(dst_addr=dst_addr)
        try:
            msg = can.Message(arbitration_id=ident, data=data, is_extended_id=True)
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"[NIUERA] Error sending CAN frame: {e}")
            return False

    def _send_set_reg_u32(self, reg: int, value_u32: int, dst_addr=None):
        data = [self.FC_SET, 0x00, (reg >> 8) & 0xFF, reg & 0xFF] + self._u32_to_be_bytes(value_u32)
        return self._send(data, dst_addr=dst_addr)

    def _send_set_reg_i32(self, reg: int, value_i32: int, dst_addr=None):
        data = [self.FC_SET, 0x00, (reg >> 8) & 0xFF, reg & 0xFF] + self._i32_to_be_bytes(value_i32)
        return self._send(data, dst_addr=dst_addr)

    def _send_read_reg(self, reg: int, dst_addr=None):
        data = [self.FC_READ, 0x00, (reg >> 8) & 0xFF, reg & 0xFF, 0, 0, 0, 0]
        return self._send(data, dst_addr=dst_addr)

    def _process_frame(self, msg):
        # NIUERA uses extended ID
        if not msg.is_extended_id:
            return

        protno, ptp, dst, src, grp = self._parse_identifier(msg.arbitration_id)
        if protno != self.PROTNO_DEFAULT:
            return

        payload = list(msg.data)
        if len(payload) < 8:
            return

        fc = payload[0]

        # Response frame
        if fc == self.FC_RESP_INT:
            err = payload[1]
            reg = (payload[2] << 8) | payload[3]
            d4, d5, d6, d7 = payload[4], payload[5], payload[6], payload[7]

            if err != self.RESP_OK:
                # Fail reply exists (F2) :contentReference[oaicite:13]{index=13}
                return

            # Parse known registers
            if reg == self.REG_READ_DC_VI:
                # byte4~5: DC voltage (0.1V), byte6~7: DC current (0.01A signed) :contentReference[oaicite:14]{index=14}
                v_raw = self._u16_be(d4, d5)        # 0.1V
                i_raw = self._s16_be(d6, d7)        # 0.01A signed
                self.evse_present_voltage = v_raw / 10.0
                self.evse_present_current = i_raw / 100.0

            elif reg == self.REG_READ_STATUS:
                # Status is 32-bit, but table defines bits; we store raw for debugging :contentReference[oaicite:15]{index=15}
                self.last_status_bits = (d4 << 24) | (d5 << 16) | (d6 << 8) | d7
                # Bits 12-13 = working mode (0 grid, 1 off-grid, 2 rectifier)
                self.last_work_mode = (self.last_status_bits >> 12) & 0x03

            # else: ignore for now

    def start(self):
        """
        Power ON command:
          reg 0x0030, value 0x00000000 = startup :contentReference[oaicite:16]{index=16}
        """
        ok = self._send_set_reg_u32(self.REG_POWER_ONOFF, 0x00000000)
        if ok:
            self.started = True
        return ok

    def stop(self):
        """
        Power OFF command:
          reg 0x0030, value 0x00010000 = shutdown :contentReference[oaicite:17]{index=17}
        """
        ok = self._send_set_reg_u32(self.REG_POWER_ONOFF, 0x00010000)
        if ok:
            self.started = False

        # Safety: set current/voltage targets to 0 (best-effort)
        try:
            self.setEvTargetCurrent(0)
            self.setEvTargetVoltage(0)
        except Exception:
            pass
        return ok

    def close(self):
        """Stops threads and closes bus (same pattern)."""
        # stop heartbeat first (so it won't send after shutdown)
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
    
    # EVSE limits
    def setEvseMaxCurrent(self, value): self.evse_max_current = value
    def setEvseMinCurrent(self, value): self.evse_min_current = value
    def setEvseMaxVoltage(self, value): self.evse_max_voltage = value
    def setEvseMinVoltage(self, value): self.evse_min_voltage = value
    def setEvseMaxPower(self, value): self.evse_max_power = value
    def setEvseDeltaVoltage(self, value): pass
    def setEvseDeltaCurrent(self, value): pass

    # EV limits
    def setEvMaxCurrent(self, value): self.ev_max_current = value
    def setEvMinCurrent(self, value): self.ev_min_current = value
    def setEvMaxVoltage(self, value): self.ev_max_voltage = value
    def setEvMinVoltage(self, value): self.ev_min_voltage = value
    def setEvMinPower(self, value): pass
    def setEvMaxPower(self, value): self.ev_max_power = value

    def setEvTargetVoltage(self, voltage):
        """
        NIUERA: Set DC link voltage register 0x0077, unit mV :contentReference[oaicite:18]{index=18}
        We'll accept volts (float/int) like your Phoenix class did.
        """
        if self.isVoltageLimitExceeded(voltage):
            return False
        mv = int(float(voltage) * 1000.0)
        return self._send_set_reg_u32(self.REG_SET_DC_LINK_V, mv)

    def setEvTargetCurrent(self, current):
        """
        NIUERA: Set DC current register 0x0079, unit mA, signed
          Rectifier mode: positive
          Inverter mode:  negative :contentReference[oaicite:19]{index=19}
        We'll accept amps (float/int).
        """
        if self.isCurrentLimitExceeded(current):
            return False
        ma = int(float(current) * 1000.0)
        return self._send_set_reg_i32(self.REG_SET_DC_CURRENT, ma)

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

    def getEvsePresentVoltage(self):
        """
        Reads DC V/I from reg 0x000F and returns cached voltage.
        Voltage unit in response is 0.1V :contentReference[oaicite:20]{index=20}
        """
        self._send_read_reg(self.REG_READ_DC_VI)
        return self.evse_present_voltage

    def getEvsePresentCurrent(self):
        """
        Reads DC V/I from reg 0x000F and returns cached current.
        Current unit in response is 0.01A signed :contentReference[oaicite:21]{index=21}
        """
        self._send_read_reg(self.REG_READ_DC_VI)
        return self.evse_present_current

    def getModuleStatusBits(self):
        """Optional helper: request + return cached status bits (reg 0x0040)."""
        self._send_read_reg(self.REG_READ_STATUS)
        return self.last_status_bits

    # ----------------------------
    # Safety checks (same logic)
    # ----------------------------
    def isVoltageLimitExceeded(self, voltage):
        return voltage > self.evse_max_voltage or (self.evse_min_voltage > 0 and voltage < self.evse_min_voltage)

    def isCurrentLimitExceeded(self, current):
        return current > self.evse_max_current or (self.evse_min_current > 0 and current < self.evse_min_current)

    def isPowerLimitExceeded(self, power):
        return power > self.evse_max_power

    def StartCanLoop(self, period_s=5.0, keep_alive_read=True):
        """
        NIUERA spec mentions shutdown if background communication interruption >10s (product spec) :contentReference[oaicite:22]{index=22}
        So we keep a simple loop that:
          - Optionally re-sends 'start' (safe if already started)
          - Optionally reads DC V/I to keep traffic alive
        """
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
                # Keep module alive / ensure it's on
                if self.started:
                    # Best-effort: read DC V/I to avoid comm timeout
                    if keep_alive_read:
                        self._send_read_reg(self.REG_READ_DC_VI)
                else:
                    # If not started, don't spam; you can still call start() externally
                    pass
            except Exception as e:
                print(f"[NIUERA HEARTBEAT ERROR] {e}")
            time.sleep(float(period_s))


if __name__ == "__main__":
    print("Initializing CanNiuera...")
    try:
        charger = CanNiuera(interface="socketcan", channel="can2", bitrate=125000)
        charger.start()
        time.sleep(1)
        charger.stop()
        charger.close()
        print("Test Complete.")
    except ImportError:
        print("Skipping test: python-can not installed.")
    except Exception as e:
        print(f"Test failed (expected if vcan0 not up): {e}")
