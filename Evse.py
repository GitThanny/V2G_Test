import time
import gc
import threading
from Whitebeet import *
from CanCharger import *
from RelayControl import *

class Evse():
    def __init__(self, iftype, iface, mac, auto_authorize=False):
        self.whitebeet = Whitebeet(iftype, iface, mac)
        print(f"WHITE-beet-EI firmware version: {self.whitebeet.version}")
        self.relay = RelayControl("P8_17")
        self.relay.turn_on()
        self.CanCharger = CanCharger()
        self.schedule = None
        self.evse_config = None
        self.auto_authorize = auto_authorize
        self.charging = False
        
        # Background update thread support
        self._poll_count = 0  # For debug timing logs
        self._update_thread = None
        self._update_running = False
        self._update_params_lock = threading.Lock()
        self._latest_charging_params = None
        self._gc_manual_collect_counter = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if hasattr(self, "whitebeet"):
            del self.whitebeet

    def __del__(self):
        if hasattr(self, "whitebeet"):
            del self.whitebeet

    def _initialize(self):
        """
        Initializes the whitebeet by setting the control pilot mode and setting the duty cycle
        to 100%. The SLAC module is also started. This one needs ~1s to 2s to be ready.
        Therefore we delay the initialization by 2s.
        """

        print("Set the CP mode to EVSE")
        self.whitebeet.controlPilotSetMode(1)
        print("Set the CP duty cycle to 100%")
        self.whitebeet.controlPilotSetDutyCycle(100)
        print("Start the CP service")
        self.whitebeet.controlPilotStart()
        print("Start SLAC in EVSE mode")
        self.whitebeet.slacStart(1)
        time.sleep(2)

    def _waitEvConnected(self, timeout):
        """
        We check for the state on the CP. When there is no EV connected we have state A on CP.
        When an EV connects the state changes to state B and we can continue with further steps.
        """
        timestamp_start = time.time()
        cp_state = self.whitebeet.controlPilotGetState()
        if cp_state == 1:
            print("EV already connected")
            return True
        elif cp_state > 1:
            print("CP in wrong state: {}".format(cp_state))
            return False
        else:
            print("Wait until an EV connects")
            while True:
                cp_state = self.whitebeet.controlPilotGetState()
                if timeout != None and timestamp_start + timeout > time.time():
                    return False
                if cp_state == 0:
                    time.sleep(0.1)
                elif cp_state == 1:
                    print("EV connected")
                    return True
                else:
                    print("CP in wrong state: {}".format(cp_state))
                    return False

    def _handleEvConnected(self):
        """
        When an EV connected we start our start matching process of the SLAC which will be ready
        to answer SLAC request for the next 50s. After that we set the duty cycle on the CP to 5%
        which indicates that the EV can start with sending SLAC requests.
        """
        self.CanCharger.start()
        print("Start SLAC matching")
        self.whitebeet.slacStartMatching()
        print("Set duty cycle to 5%")
        self.whitebeet.controlPilotSetDutyCycle(5)
        try:
            if self.whitebeet.slacMatched() == True:
                print("SLAC matching successful")
                self._handleNetworkEstablished()
                return True
            else:
                print("SLAC matching failed")
                return False
        except TimeoutError as e:
            print(e)
            return False

    def _handleNetworkEstablished(self):
        """
        When SLAC was successful we can start the V2G module. Set our supported protocols,
        available payment options and energy transfer modes. Then we start waiting for
        notifications for requested parameters.
        """
        self.CanCharger.start()
        print("Set V2G mode to EVSE")
        self.whitebeet.v2gSetMode(1)

        self.evse_config = {
            "evse_id_DIN": '+49*123*456*789',
            "evse_id_ISO": 'DE*A23*E45B*78C',
            "protocol": [0, 1], 
            "payment_method": [0],
            "energy_transfer_mode": [0, 1, 2, 3, 4, 5],
            "certificate_installation_support": False,
            "certificate_update_support": False,
        }
        self.whitebeet.v2gEvseSetConfiguration(self.evse_config)

        self.dc_charging_parameters = {
            'isolation_level': 1,
            'min_voltage': self.CanCharger.getEvseMinVoltage(),
            'min_current': self.CanCharger.getEvseMinCurrent(),
            'max_voltage': self.CanCharger.getEvseMaxVoltage(),
            'max_current': self.CanCharger.getEvseMaxCurrent(),
            'max_power': self.CanCharger.getEvseMaxPower(),
            'peak_current_ripple': int(self.CanCharger.getEvseDeltaCurrent()),
            'status': 0
        }
        self.whitebeet.v2gEvseSetDcChargingParameters(self.dc_charging_parameters)

        self.ac_charging_parameters = {
            'rcd_status': 0,
            'nominal_voltage': self.CanCharger.getEvseMaxVoltage(),
            'max_current': self.CanCharger.getEvseMaxCurrent(),
        }
        self.whitebeet.v2gEvseSetAcChargingParameters(self.ac_charging_parameters)

        time.sleep(0.1)
        print("Start V2G")
        self.whitebeet.v2gEvseStartListen()
        while True:
            if self.charging:
                
                # Disable automatic GC during charging to prevent random pauses
                if gc.isenabled():
                    gc.disable()
                    print("[GC] Automatic garbage collection disabled during charging")

                loop_start = time.time()
                recv_start = time.time()
                
                id, data = self.whitebeet.v2gEvseReceiveRequestSilent()
                
                recv_time = (time.time() - recv_start) * 1000

                charging_parameters = {
                    'isolation_level': 1,
                    'present_voltage': int(self.CanCharger.getEvsePresentVoltage()),
                    'present_current': int(self.CanCharger.getEvsePresentCurrent()),
                    'max_voltage': int(self.CanCharger.getEvseMaxVoltage()),
                    'max_current': int(self.CanCharger.getEvseMaxCurrent()),
                    'max_power': int(self.CanCharger.getEvseMaxPower()),
                    'status': 0,
                }

                                # ===== DEBUG TIMING: Update =====
                update_start = time.time()
                # Use FAST version - fire and forget, no waiting
                update_ok = self.whitebeet.v2gEvseUpdateDcChargingParametersFast(charging_parameters)
                if not update_ok:
                    print("[UPDATE] Fast update failed, retrying with normal method")
                    try:
                        self.whitebeet.v2gEvseUpdateDcChargingParameters(charging_parameters)
                    except:
                        pass
                update_time = (time.time() - update_start) * 1000

                # Timing monitoring - ALWAYS log for debugging
                total_time = (time.time() - loop_start) * 1000
                self._poll_count += 1
                
                
                # NOTE: GC is DISABLED during charging (see line 148)
                # DO NOT call gc.collect() here - it takes 832ms and causes Session Error 23!
                # Memory can safely grow for a 30-60 min charging session
                
                # Log EVERY iteration until we find the problem
                # print(f"[LOOP#{self._poll_count}] recv:{recv_time:.0f}ms upd:{update_time:.0f}ms total:{total_time:.0f}ms id:{id}")
                
                if total_time > 400:
                    print(f"[TIMING WARNING] Loop took {total_time:.0f}ms (should be < 500ms)")

            else:
                id, data = self.whitebeet.v2gEvseReceiveRequest()

            if id == None or data == None:
                pass
            elif id == 0x80:
                self._handleSessionStarted(data)
            elif id == 0x81:
                self._handlePaymentSelected(data)
            elif id == 0x82:
                self._handleRequestAuthorization(data)
            elif id == 0x83:
                self._handleEnergyTransferModeSelected(data)
            elif id == 0x84:
                self._handleRequestSchedules(data)
            elif id == 0x85:
                self._handleDCChargeParametersChanged(data)
            elif id == 0x86:
                self._handleACChargeParametersChanged(data)
            elif id == 0x87:
                self._handleRequestCableCheck(data)
            elif id == 0x88:
                self._handlePreChargeStarted(data)
            elif id == 0x89:
                self._handleRequestStartCharging(data)
            elif id == 0x8A:
                self._handleRequestStopCharging(data)
            elif id == 0x8B:
                self._handleWeldingDetectionStarted(data)
            elif id == 0x8C:
                self._handleSessionStopped(data)
                break
            elif id == 0x8D:
                pass
            elif id == 0x8E:
                self._handleSessionError(data)
            elif id == 0x8F:
                self._handleCertificateInstallationRequested(data)
            elif id == 0x90:
                self._handleCertificateUpdateRequested(data)
            elif id == 0x91:
                self._handleMeteringReceiptStatus(data)
            else:
                print("Message ID not supported: {:02x}".format(id))
                break
        self.whitebeet.v2gEvseStopListen()

    def _handleSessionStarted(self, data):
        """
        Handle the SessionStarted notification
        """
        self.CanCharger.start()
        print("\"Session started\" received")
        message = self.whitebeet.v2gEvseParseSessionStarted(data)
        print("Protocol: {}".format(message['protocol']))
        print("Session ID: {}".format(message['session_id'].hex()))
        print("EVCC ID: {}".format(message['evcc_id'].hex()))
        
        time.sleep(2)

    def _handlePaymentSelected(self, data):
        """
        Handle the PaymentSelected notification
        """
        print("\"Payment selcted\" received")
        message = self.whitebeet.v2gEvseParsePaymentSelected(data)
        print("Selected payment method: {}".format(message['selected_payment_method']))
        if message['selected_payment_method'] == 1:
            print("Contract certificate: {}".format(message['contract_certificate'].hex()))
            print("mo_sub_ca1: {}".format(message['mo_sub_ca1'].hex()))
            print("mo_sub_ca2: {}".format(message['mo_sub_ca2'].hex()))
            print("EMAID: {}".format(message['emaid'].hex()))
            
        time.sleep(2)

    def _handleRequestAuthorization(self, data):
        """
        Handle the RequestAuthorization notification.
        The authorization status will be requested from the user.
        """
        print("\"Request Authorization\" received")
        message = self.whitebeet.v2gEvseParseAuthorizationStatusRequested(data)
        print(message['timeout'])

        if self.auto_authorize:
            print("Vehicle was authorized automatically via --auto flag!")
            try:
                self.whitebeet.v2gEvseSetAuthorizationStatus(True)
            except Warning as e:
                print("Warning: {}".format(e))
            except ConnectionError as e:
                print("ConnectionError: {}".format(e))
            return

        timeout = int(message['timeout'] / 1000) - 1
        # Promt for authorization status
        auth_str = input("Authorize the vehicle? Type \"yes\" or \"no\" in the next {}s: ".format(timeout))
        authorized = auth_str is not None and auth_str.lower() == "yes"

        print(f"Vehicle was {'authorized' if authorized else 'NOT authorized'} by user!")
        try:
            self.whitebeet.v2gEvseSetAuthorizationStatus(authorized)
        except (Warning, ConnectionError) as e:
            print(f"{type(e).__name__}: {e}")
            
        time.sleep(2)

    def _handleEnergyTransferModeSelected(self, data):
        """
        Handle the energy transfer mode selected notification
        """
        print("\"Energy transfer mode selected\" received")

        message = self.whitebeet.v2gEvseParseEnergyTransferModeSelected(data)

        if 'departure_time' in message:        
            print('Departure time: {}'.format(message['departure_time']))

        if 'energy_request' in message:
            print('Energy request: {}'.format(message['energy_request']))

        print('Maximum voltage: {}'.format(message['max_voltage']))
        self.CanCharger.setEvMaxVoltage(message['max_voltage'])

        if 'min_current' in message:
            print('Minimum current: {}'.format(message['min_current']))
            self.CanCharger.setEvMinCurrent(message['min_current'])

        print('Maximum current: {}'.format(message['max_current']))
        self.CanCharger.setEvMaxCurrent(message['max_current'])

        if 'max_power' in message:
            print('Maximum power: {}'.format(message['max_power']))
            self.CanCharger.setEvMaxPower(message['max_power'])

        if 'energy_capacity' in message:
            print('Energy Capacity: {}'.format(message['energy_capacity']))

        if 'full_soc' in message:
            print('Full SoC: {}'.format(message['full_soc']))

        if 'bulk_soc' in message:
            print('Bulk SoC: {}'.format(message['bulk_soc']))

        if 'ready' in message:
            print('Ready: {}'.format('yes' if message['ready'] else 'no'))

        if 'error_code' in message:
            print('Error code: {}'.format(message['error_code']))

        if 'soc' in message:
            print('SoC: {}'.format(message['soc']))

        if 'selected_energy_transfer_mode' in message:
            print('Selected energy transfer mode: {}'.format(message['selected_energy_transfer_mode']))
            if not message['selected_energy_transfer_mode'] in self.evse_config['energy_transfer_mode']:
                print('Energy transfer mode mismatch!')
                try:
                    self.whitebeet.v2gEvseStopCharging()
                except Warning as e:
                    print("Warning: {}".format(e))
                except ConnectionError as e:
                    print("ConnectionError: {}".format(e))
                    
        time.sleep(0.2)

    def _handleRequestSchedules(self, data):
        """
        Handle the RequestSchedules notification
        """
        print("\"Request Schedules\" received")
        message = self.whitebeet.v2gEvseParseSchedulesRequested(data)
        print("Max entries: {}".format(message['max_entries']))
        maxEntry = max([len(self.schedule), message['max_entries']])
        print("Set the schedule: {}".format(self.schedule))
        try:
            self.whitebeet.v2gEvseSetSchedules(self.schedule)
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))
            
        time.sleep(0.2)
        
    def _handleDCChargeParametersChanged(self, data):
        """
        Handle the DCChargeParametersChanged notification
        """
        print("\"DC Charge Parameters Changed\" received")
        message = self.whitebeet.v2gEvseParseDCChargeParametersChanged(data)

        print("EV maximum current: {}A".format(message['max_current']))
        self.CanCharger.setEvMaxCurrent(message['max_current'])

        print("EV maximum voltage: {}V".format(message['max_voltage']))
        self.CanCharger.setEvMaxVoltage(message['max_voltage'])

        if 'max_power' in message:
            print("EV maximum power: {}W".format(message['max_power']))
            self.CanCharger.setEvMaxPower(message['max_power'])

        print('EV ready: {}'.format(message['ready']))
        print('Error code: {}'.format(message['error_code']))
        print("SOC: {}%".format(message['soc']))

        if 'target_voltage' in message:
            print("EV target voltage: {}V".format(message['target_voltage']))
            self.CanCharger.setEvTargetVoltage(message['target_voltage'])

        if 'target_current' in message:
            print("EV target current: {}A".format(message['target_current']))
            self.CanCharger.setEvTargetCurrent(message['target_current'])
        
        if 'charging_complete' in message:
            print("Charging complete: {}".format(message['charging_complete']))
        if 'bulk_charging_complete' in message:
            print("Bulk charging complete: {}".format(message['bulk_charging_complete']))
        if 'remaining_time_to_full_soc' in message:
            print("Remaining time to full SOC: {}s".format(message['remaining_time_to_full_soc']))
        if 'remaining_time_to_bulk_soc' in message:
            print("Remaining time to bulk SOC: {}s".format(message['remaining_time_to_bulk_soc']))
    
        charging_parameters = {
            'isolation_level': 1,
            'present_voltage': int(self.CanCharger.getEvsePresentVoltage()),
            'present_current': int(self.CanCharger.getEvsePresentCurrent()),
            'max_voltage': int(self.CanCharger.getEvseMaxVoltage()),
            'max_current': int(self.CanCharger.getEvseMaxCurrent()),
            'max_power': int(self.CanCharger.getEvseMaxPower()),
            'status': 0,
        }

        try:
            self.whitebeet.v2gEvseUpdateDcChargingParameters(charging_parameters)
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))

    def _handleACChargeParametersChanged(self, data):
        """
        Handle the ACChargeParametersChanged notification
        """
        print("\"AC Charge Parameters Changed\" received")
        message = self.whitebeet.v2gEvseParseACChargeParametersChanged(data)

        print("EV maximum voltage: {}V".format(message['max_voltage']))
        self.CanCharger.setEvMaxVoltage(message['max_voltage'])

        print("EV minimum current: {}W".format(message['min_current']))
        self.CanCharger.setEvMinCurrent(message['min_current'])

        print("EV maximum current: {}A".format(message['max_current']))
        self.CanCharger.setEvMaxCurrent(message['max_current'])

        print("Energy amount: {}A".format(message['energy_amount']))    

        charging_parameters = {
            'rcd_status': 0,
            'max_current': int(self.CanCharger.getEvseMaxCurrent()),
        }

        try:
            self.whitebeet.v2gEvseUpdateAcChargingParameters(charging_parameters)
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))

    def _handleRequestCableCheck(self, data):
        """
        Handle the RequestCableCheck notification
        """
        self.charging = True
        self.relay.turn_on()
        self.CanCharger.start()
        print("\"Request Cable Check Status\" received")
        self.whitebeet.v2gEvseParseCableCheckRequested(data)
        try:
            self.whitebeet.v2gEvseSetCableCheckFinished(True)
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))

    def _handlePreChargeStarted(self, data):
        """
        Handle the PreChargeStarted notification
        """
        self.CanCharger.start()
        print("\"Pre Charge Started\" received")
        self.whitebeet.v2gEvseParsePreChargeStarted(data)

    def _handleRequestStartCharging(self, data):
        """
        Handle the StartChargingRequested notification
        """
        print("\"Start Charging Requested\" received")
        message = self.whitebeet.v2gEvseParseStartChargingRequested(data)
        print("Schedule tuple ID: {}".format(message['schedule_tuple_id']))
        print("Charging profiles: {}".format(message['charging_profiles']))
        
        # Start fast polling loop ONLY when actual charging starts
        self.charging = True
        self._poll_count = 0  # Reset poll count for this session
        
        self.CanCharger.start()
        try:
            self.whitebeet.v2gEvseStartCharging()
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))

    def _handleRequestStopCharging(self, data):
        """
        Handle the RequestStopCharging notification
        """
        self.relay.turn_off()
        print("\"Request Stop Charging\" received")
        message = self.whitebeet.v2gEvseParseStopChargingRequested(data)
        print('Timeout: {}'.format(message['timeout']))
        print('Timeout: {}'.format('yes' if message['renegotiation'] else 'no'))
        self.CanCharger.stop()
        try:
            self.whitebeet.v2gEvseStopCharging()
        except Warning as e:
            print("Warning: {}".format(e))
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))

    def _handleWeldingDetectionStarted(self, data):
        """
        Handle the WeldingDetectionStarted notification
        """
        print("\"Welding Detection Started\" received")
        self.whitebeet.v2gEvseParseWeldingDetectionStarted(data)

    def _handleSessionStopped(self, data):
        """
        Handle the SessionStopped notification
        """
        self.relay.turn_off()
        self.charging = False
        print("\"Session stopped\" received")
        message = self.whitebeet.v2gEvseParseSessionStopped(data)
        print('Closure type: {}'.format(message['closure_type']))
        self.CanCharger.stop()

    def _handleSessionError(self, data):
        """
        Handle the SessionError notification
        """
        self.relay.turn_off()
        print("\"Session Error\" received")
        self.charging = False
        message = self.whitebeet.v2gEvseParseSessionError(data)
        self.CanCharger.stop()

        error_messages = {
            0: 'Unspecified',
            1: 'Sequence error',
            2: 'Service ID invalid',
            3: 'Unknown session',
            4: 'Service selection invalid',
            5: 'Payment selection invalid',
            6: 'Certificate expired',
            7: 'Signature Error',
            8: 'No certificate available',
            9: 'Certificate chain error',
            10: 'Challenge invalid',
            11: 'Contract canceled',
            12: 'Wrong charge parameter',
            13: 'Power delivery not applied',
            14: 'Tariff selection invalid',
            15: 'Charging profile invalid',
            16: 'Present voltage too low',
            17: 'Metering signature not valid',
            18: 'No charge service selected',
            19: 'Wrong energy transfer type',
            20: 'Contactor error',
            21: 'Certificate not allowed at this EVSE',
            22: 'Certificate revoked',
            23: 'Charge parameter timeout reached'
        }

        print('Session error: {}: {}'.format(message['error_code'], error_messages[message['error_code']]))
        try:
            self.whitebeet.v2gEvseStopCharging()
        except Warning as e:
            print("Warning: {}".format(e))
            self.whitebeet.v2gEvseStopListen()
        except ConnectionError as e:
            print("ConnectionError: {}".format(e))
            self.whitebeet.v2gEvseStopListen()

    def _handleCertificateInstallationRequested(self, data):
        """
        Handle the CertificateInstallationRequested notification
        """
        print("\"Certificate Installation Requested\" received")
        message = self.whitebeet.v2gEvseParseCertificateInstallationRequested(data)
        print('Timeout: {}'.format(message['timeout']))
        print('EXI request: {}'.format(message['exi_request']))

        status = 2
        certificationResponse = []

    def _handleCertificateUpdateRequested(self, data):
        """
        Handle the CertificateUpdateRequested notification
        """
        print("\"Certificate Update Requested\" received")
        message = self.whitebeet.v2gEvseParseCertificateUpdateRequested(data)
        print('Timeout: {}'.format(message['timeout']))
        print('EXI request: {}'.format(message['exi_request']))

        status = 2
        certificationResponse = []
            
    def _handleMeteringReceiptStatus(self, data):
        """
        Handle the MeteringReceiptStatus notification
        """
        print("\"Metering Receipt Status\" received")
        message = self.whitebeet.v2gEvseParseMeteringReceiptStatus(data)
        print('Metering receipt status: {}'.format('verified' if message['status'] == True else 'not verified'))

    def getCanCharger(self):
        """
        Returns the CanCharger object
        """
        if hasattr(self, "CanCharger"):
            return self.CanCharger
        else:
            return None

    def getWhitebeet(self):
        """
        Returns the whitebeet object
        """
        if hasattr(self, "whitebeet"):
            return self.whitebeet
        else:
            return None

    def setSchedule(self, schedule):
        """
        Sets the schedule. This schedule will be used when the whitebeet requests this data
        """
        if isinstance(schedule, dict) == False:
            print("Schedule needs to be of type dict")
            return False
        else:
            self.schedule = schedule
            return True

    def loop(self):
        """
        This will handle a complete charging session of the EVSE.
        """
        self._initialize()
        if self._waitEvConnected(None):
            return self._handleEvConnected()
        else:
            return False


