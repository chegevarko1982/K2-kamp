"""K2 Plus 2.4: one optical baseline and durable, job-bound mesh recovery.

All motion is delegated to reviewed stock commands. Never probe on recovery.
"""
import hashlib
import json
import logging
import math
import os
import struct
import uuid


EXPECTED_SOURCES = {'/usr/share/klipper/klippy/extras/base_info.py': '35cea1f0a2cf1e2f13590da2ae6c0ae7bdbce7cab1c289ad9504ab0927fec659', '/usr/share/klipper/klippy/extras/bl24c16f.py': '3314731819318b632294c2cbacbfaa04cc1e8a9e9a33370212ed9417c2686168', '/usr/share/klipper/klippy/extras/custom_macro.py': '3025bf4e824c15d0c87cd15aac83adf3303aac4e8458c97ab3febc8d17b57aa2', '/usr/share/klipper/klippy/extras/gcode_move.py': 'a49086bfacdaa96ba8c3c4892b64c1687cf880f350643a4e903bd36d9a36b300', '/usr/share/klipper/klippy/extras/homing.py': 'cc847821cb60fb0a322e75685ea2c27e86d608a15ef4888a0cbe3dce16a2f765', '/usr/share/klipper/klippy/extras/print_stats.py': '3850151743659b22b61ec6daf013948c6de799de085138029d547b023d0d7341', '/usr/share/klipper/klippy/extras/stepper_enable.py': '2bd12d34dd1468bc9d8cc89f65d76c4f0de7dca9d896bd96b38b0a3647267517', '/usr/share/klipper/klippy/extras/virtual_sdcard.py': 'c39c839706c4152828e144dda1fc23f7bdaacc426dc8241e77ac9d90235d54c6', '/usr/share/klipper/klippy/extras/z_align.py': '4ebe29d7957c6e42d05f95d1fe9a5e40c4d9953fe70c3b7470914a7d1b8b2b1c', '/usr/share/klipper/klippy/extras/z_tilt.py': '846bc1255ea48bd6501e695ccfe0f7b30a4265fd5ec8f9fba47d675207b2b31b', '/usr/share/klipper/klippy/stepper.py': '76271cf678b8ce627f77978d7b2fc27b20f58ae9eae38bcd400626036753fea2', '/usr/share/klipper/klippy/extras/bed_mesh.py': '00a6c3c2da5938b28e29f562d07966c6854bf3657104245e8709c8a5d42ebec0', '/usr/share/klipper/klippy/extras/probe.py': 'd6917fc49e31088096151291d89c0f69292362bbc8d36ac16ef8f767d1cad0a4', '/usr/share/klipper/klippy/gcode.py': 'fa0d037602a58d40d8694422ade76613961f2a1b6a71f61668b76a59c34680d5', '/usr/share/klipper/klippy/extras/homing_override.py': 'c11b4cdf7db5cb59037a107a09783c0dd0d89057e9d9a4e84c0c3e22013f632b', '/mnt/UDISK/printer_data/config/sensorless.cfg': '9d635c7410d49325fbca7f70108451fbac39f6d5b3fed06e49a9507934764a55'}


class RequestRefused(Exception):
    """An external refused request must not abort an existing valid print."""


def canonical(data):
    return json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_json(path, limit=2**20):
    with open(path, 'rb') as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError('KAMP state exceeds size limit')
    return json.loads(data)


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_json(path, data):
    parent = os.path.dirname(path)
    os.makedirs(parent, mode=0o700, exist_ok=True)
    raw = canonical(data)
    temp = path + '.tmp-' + uuid.uuid4().hex
    try:
        with open(temp, 'xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
        sync_dir(parent)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def envelope(data):
    return {'payload': data, 'sha256': digest(canonical(data))}


def checked_envelope(path):
    value = read_json(path)
    if set(value) != {'payload', 'sha256'} or digest(canonical(value['payload'])) != value['sha256']:
        raise ValueError('KAMP snapshot checksum mismatch')
    return value['payload']


class Recovery:
    def _configure_recovery(self, config):
        self.state_dir = config.get('state_directory', '/mnt/UDISK/printer_data/kamp_state')
        self.checkpoint_path = os.path.join(self.state_dir, 'checkpoint.json')
        self.transaction_path = os.path.join(self.state_dir, 'mechanics-transaction.json')
        self.eeprom_path = os.path.join(self.state_dir, 'eeprom-generation.json')
        self.recovering = self.recovery_optical_done = False
        self.preparation_ready = self._preparing = self._hot_running = False
        self._optical_running = False
        self._optical_sequence = 0
        self._optical_reference = None
        self._cold_datum = self._pending_adjustment = None
        self.job_id = None
        self._selected_job = None
        self._recovery_data = None
        self._mechanical_generation = None
        self._lan_home_deadline = None
        commands = {
            'KAMP_LAN_PREPARE': self.cmd_LAN_PREPARE,
            'KAMP_ROUGH_HOME': self.cmd_ROUGH_HOME,
            'KAMP_HOT_PREPARE': self.cmd_HOT_PREPARE,
            'KAMP_START_FILE': self.cmd_START_FILE,
            'KAMP_ZDOWN': self.cmd_ZDOWN,
            'KAMP_NOTE_Z_CHANGE': self.cmd_NOTE_Z_CHANGE,
            'KAMP_CLEAR_MESH': self.cmd_CLEAR_MESH,
            'KAMP_OFFSET_MESH': self.cmd_OFFSET_MESH,
            'KAMP_TILT': self.cmd_TILT,
            'KAMP_ADJUST': self.cmd_ADJUST,
        }
        for name, callback in commands.items():
            self.gcode.register_command(name, self._guard(callback))
        self.printer.register_event_handler('stepper_enable:motor_off', self._motor_off)

    def _init_recovery(self):
        from extras.z_align import Zalign
        from extras.homing import PrinterHoming
        from extras.z_tilt import RetryHelper, ZTilt, ZAdjustHelper
        from extras.custom_macro import CUSTOM_MACRO
        from extras.probe import ProbePointsHelper
        from stepper import MCU_stepper
        from extras.gcode_move import GCodeMove
        from extras.homing_override import HomingOverride
        if '/usr/share/klipper/klippy/stepper.py' not in EXPECTED_SOURCES:
            raise ValueError('runtime stepper.py must be reviewed before installation')
        for path, expected in EXPECTED_SOURCES.items():
            with open(path, 'rb') as stream:
                if digest(stream.read()) != expected:
                    raise ValueError('unreviewed firmware source: ' + path)
        self.za = self.printer.lookup_object('z_align')
        self.zt = self.printer.lookup_object('z_tilt')
        self.hm = self.printer.lookup_object('homing')
        self.gm = self.printer.lookup_object('gcode_move')
        self.sd = self.printer.lookup_object('virtual_sdcard')
        self.home_override = self.printer.lookup_object('homing_override')
        registered_home = self._dispatch_target(self.gcode.ready_gcode_handlers['G28'])
        if (getattr(self.home_override.cmd_G28, '__func__', None) is not HomingOverride.cmd_G28
                or getattr(self.home_override.cmd_G28, '__self__', None) is not self.home_override
                or getattr(registered_home, '__func__', None) is not HomingOverride.cmd_G28
                or getattr(registered_home, '__self__', None) is not self.home_override
                or getattr(self.home_override.prev_G28, '__func__', None) is not PrinterHoming.cmd_G28
                or getattr(self.home_override.prev_G28, '__self__', None) is not self.hm):
            raise ValueError('unreviewed G28 override/underlying dispatcher')
        self._stock_g28 = registered_home
        if self.za.pin_len != 2:
            raise ValueError('KAMP recovery requires the reviewed dual-Z layout')
        checks = ((self.za.cmd_ZDOWN, Zalign.cmd_ZDOWN),
                  (self.hm.write_real_zmax, PrinterHoming.write_real_zmax),
                  (self.zt.retry_helper.write_z_adjustment, RetryHelper.write_z_adjustment),
                  (self.gm.cmd_CX_RESTORE_GCODE_STATE, GCodeMove.cmd_CX_RESTORE_GCODE_STATE))
        if any(getattr(bound, '__func__', None) is not function for bound, function in checks):
            raise ValueError('unreviewed mechanical/recovery callback')
        extra = ((self.zt.cmd_Z_TILT_ADJUST,ZTilt.cmd_Z_TILT_ADJUST),
                 (self.zt.z_helper.adjust_steppers,ZAdjustHelper.adjust_steppers),
                 (self.zt.probe_helper.finalize_callback,ZTilt.probe_finalize),
                 (self.zt.probe_helper.start_probe,ProbePointsHelper.start_probe),
                 (self._dispatch_target(self.gcode.ready_gcode_handlers['_KAMP_TILT_BASE']),ZTilt.cmd_Z_TILT_ADJUST),
                 (self._dispatch_target(self.gcode.ready_gcode_handlers['_KAMP_ROUGH_BASE']),CUSTOM_MACRO.cmd_CX_ROUGH_G28))
        if any(getattr(bound,'__func__',None) is not function for bound,function in extra):
            raise ValueError('unreviewed rough-home or tilt dispatch')
        for stepper in self._toolhead().get_kinematics().get_steppers():
            if stepper.get_name() not in ('stepper_z','stepper_z1'):continue
            for name in ('_query_mcu_position','get_mcu_position','set_position','get_step_dist'):
                if getattr(getattr(stepper,name),'__func__',None) is not getattr(MCU_stepper,name):
                    raise ValueError('unreviewed Z counter method: '+name)
        self._stock_zdown = self.za.cmd_ZDOWN
        self._stock_datum = self.hm.write_real_zmax
        self._stock_adjustment = self.zt.retry_helper.write_z_adjustment
        self._stock_restore = self.gm.cmd_CX_RESTORE_GCODE_STATE
        # Homing looks up cmd_ZDOWN dynamically; registered public ZDOWN is
        # redirected by its macro to KAMP_ZDOWN, reaching this same wrapper.
        self.za.cmd_ZDOWN = self._checked_zdown
        self.hm.write_real_zmax = self._stage_datum
        self.zt.retry_helper.write_z_adjustment = self._stage_adjustment
        self.gm.cmd_CX_RESTORE_GCODE_STATE = self._restore_stock
        # G28 is traditional G-code: preserve bare X/Y/Z parameters by wrapping
        # its Python callback, rather than routing through an extended macro.
        self.gcode.register_command('G28', None)
        guarded_home = self._guard(self._dispatch_home)
        self.gcode.register_command('G28', guarded_home)
        self.home_override.cmd_G28 = guarded_home
        # ZMesh registers these commands dynamically on its first instance.
        # Intercept after default has loaded, before any print can begin.
        for name in ('BED_MESH_SET_DISABLE','BED_MESH_SET_ENABLE'):
            callback=self.gcode.register_command(name,None)
            if callback is None:
                raise ValueError('missing mesh enable/disable command: '+name)
            def guarded(gcmd, original=callback):
                if self.active or self.recovering or self.armed or self.preparation_ready:
                    raise RequestRefused('mesh enable changes are blocked during KAMP')
                self._invalidate_checkpoint()
                return original(gcmd)
            self.gcode.register_command(name,self._guard(guarded))
        self._repair_mechanics()

    def _dispatch_target(self, callback):
        # Extended commands and rename_existing add dispatcher lambdas. Unwrap
        # only the exact reviewed register_command closure, never arbitrary
        # wrappers supplied by another module.
        from gcode import GCodeDispatch
        codes=[c for c in GCodeDispatch.register_command.__code__.co_consts
               if hasattr(c,'co_name') and c.co_name=='<lambda>']
        for _ in range(4):
            if getattr(callback,'__func__',None) is not None:return callback
            if getattr(callback,'__code__',None) not in codes:
                raise ValueError('unreviewed G-code dispatcher wrapper')
            cells=dict(zip(callback.__code__.co_freevars,callback.__closure__))
            if cells['self'].cell_contents is not self.gcode:
                raise ValueError('G-code wrapper belongs to another dispatcher')
            callback=cells['origfunc'].cell_contents
        raise ValueError('excessive G-code dispatcher wrapper depth')

    def _toolhead(self):
        return self.printer.lookup_object('toolhead')

    def _settle(self):
        toolhead = self._toolhead()
        toolhead.wait_moves()
        toolhead.flush_step_generation()

    def _z_positions(self):
        self._settle()
        steppers = {s.get_name(): s for s in self._toolhead().get_kinematics().get_steppers()}
        result = {}
        for name in ('stepper_z', 'stepper_z1'):
            stepper = steppers[name]
            # Synchronize actual normalized MCU counters, including invert_dir.
            stepper._query_mcu_position()
            result[name] = (stepper.get_mcu_position(), stepper.get_step_dist())
        return result

    def _flags(self, value=False):
        self._toolhead().G29_flag = bool(value)
        self.za.zdown_switch_enable = 0

    def _invalidate_preparation(self):
        self._lan_home_deadline = None
        self.preparation_ready = self._preparing = self._hot_running = False
        self._optical_reference = None
        self._mechanical_generation = None
        if hasattr(self, 'za'):
            self._flags(False)

    def _invalidate_checkpoint(self):
        for path in (self.checkpoint_path,self.eeprom_path):
            if os.path.exists(path):
                os.unlink(path)
                sync_dir(self.state_dir)
        self.recovering = self.recovery_optical_done = False
        self._recovery_data = None

    def _motor_off(self, eventtime):
        self._invalidate_preparation()
        if self.active or self.recovering:
            self._safe_stop()

    def cmd_NOTE_Z_CHANGE(self, gcmd):
        name = gcmd.get('STEPPER', '')
        if name not in ('stepper_z', 'stepper_z1') or gcmd.get('ENABLE', '') == '1':
            return
        if self.active or self.recovering:
            raise RequestRefused('manual Z change is blocked during a KAMP job')
        self._invalidate_preparation()
        self._invalidate_checkpoint()

    def _checked_zdown(self, gcmd):
        if self.active and not self.recovering:
            raise ValueError('optical ZDOWN is blocked during an active mesh session')
        self.preparation_ready = False
        self._optical_reference = None
        had_error = []
        original_error = self.gcode._respond_error
        def record_error(message):
            had_error.append(message)
            return original_error(message)
        self.gcode._respond_error = record_error
        self._optical_running = True
        try:
            result = self._stock_zdown(gcmd)
            status = self.za.mcu._serial.z_align_status
            if (result != 0 or had_error or self.za.cur_retries >= self.za.retries
                    or status.get('flag') != 1
                    or abs(status.get('deltaError1', float('inf'))) >= self.za.retry_tolerance):
                raise ValueError('optical ZDOWN did not complete successfully')
            self._optical_sequence += 1
            if self._preparing and self._toolhead().G29_flag:
                self._optical_reference = self._z_positions()
            if self.recovering:
                if self._toolhead().G29_flag:
                    raise ValueError('recovery optical used a calibration mode')
                self.recovery_optical_done = True
            return result
        finally:
            self._optical_running = False
            self.gcode._respond_error = original_error

    def cmd_CLEAR_MESH(self, gcmd):
        if self.active and not (self.probing or self.recovering):
            raise RequestRefused('mesh clearing is blocked during an active KAMP job')
        self.gcode.run_script_from_command('_KAMP_MESH_CLEAR_BASE '+gcmd.get_raw_command_parameters())

    def cmd_OFFSET_MESH(self, gcmd):
        if self.active or self.recovering or self.armed or self.preparation_ready:
            raise RequestRefused('mesh offsets are blocked during KAMP preparation/job')
        self._invalidate_checkpoint()
        self.gcode.run_script_from_command('_KAMP_MESH_OFFSET_BASE '+gcmd.get_raw_command_parameters())

    def cmd_TILT(self, gcmd):
        if self._hot_running or self.manual_busy:
            self.gcode.run_script_from_command('_KAMP_TILT_BASE '+gcmd.get_raw_command_parameters())
            return
        if self.active or self.recovering:
            raise RequestRefused('external tilt is blocked during a KAMP job')
        self._invalidate_preparation()
        self._invalidate_checkpoint()
        self.gcode.run_script_from_command('_KAMP_TILT_BASE '+gcmd.get_raw_command_parameters())

    def cmd_ADJUST(self, gcmd):
        if not self._optical_running:
            if self.active or self.recovering:
                raise RequestRefused('external Z adjustment is blocked during a KAMP job')
            self._invalidate_preparation()
            self._invalidate_checkpoint()
        self.gcode.run_script_from_command('_KAMP_ADJUST_BASE '+gcmd.get_raw_command_parameters())

    def cmd_ZDOWN(self, gcmd):
        self._require_ready(gcmd)
        return self.za.cmd_ZDOWN(gcmd)

    def _stage_datum(self, value):
        if self._preparing:
            self._cold_datum = float(value)  # Deliberately not published as hot datum.
        else:
            self._stock_datum(value)

    def _stage_adjustment(self):
        values = {'stepper_z_adjustment': float(self.zt.stepper_z_adjustment),
                  'stepper_z1_adjustment': float(self.zt.stepper_z1_adjustment)}
        if self._hot_running:
            self._pending_adjustment = values
        else:
            atomic_json(self.zt.stepper_adjustment_path, values)

    def cmd_LAN_PREPARE(self, gcmd):
        self._require_ready(gcmd)
        state = self.printer.lookup_object('print_stats').get_status(0)['state']
        if (self.active or self.recovering or self.armed or self.manual_busy
                or self._preparing or self._hot_running or self._optical_running
                or state in ('printing', 'paused')):
            raise RequestRefused('LAN preparation is not allowed during this job')
        wait = gcmd.get_int('WAIT_TEMP', 0)
        if wait not in (0, 1):
            raise ValueError('invalid LAN WAIT_TEMP')
        if wait:
            if not self.preparation_ready:
                raise ValueError('LAN homing did not create a KAMP optical baseline')
            return
        self._invalidate_checkpoint()
        self._invalidate_preparation()
        # The actual daemon sends PRINT_TEMP_SET followed immediately by G28 Z.
        # A stale marker must never turn a later manual/recovery home into prep.
        self._lan_home_deadline = self.printer.get_reactor().monotonic() + 120.0
        logging.info('KAMP 2.4.1: LAN homing authorization issued')

    def _dispatch_home(self, gcmd):
        if self.home_override.in_script or self._preparing or self._hot_running or self.recovering:
            return self._stock_g28(gcmd)
        params = gcmd.get_command_parameters()
        homes_z = 'Z' in params or not any(axis in params for axis in 'XYZ')
        if self._lan_home_deadline is None or not homes_z:
            return self._stock_g28(gcmd)
        deadline = self._lan_home_deadline
        self._lan_home_deadline = None  # Consume before any physical action.
        if self.printer.get_reactor().monotonic() > deadline:
            raise ValueError('LAN homing authorization expired; restart preparation')
        logging.info('KAMP 2.4.1: entering LAN optical preparation via G28')
        return self._run_rough(gcmd, lambda: self._stock_g28(gcmd))

    def cmd_ROUGH_HOME(self, gcmd):
        original = gcmd.get_int('ORIGINAL', 0)
        command = '_KAMP_ROUGH_BASE ' + gcmd.get_raw_command_parameters() if original else 'G28'
        return self._run_rough(gcmd, lambda: self.gcode.run_script_from_command(command), original)

    def _run_rough(self, gcmd, home, original=0):
        self._require_ready(gcmd)
        state = self.printer.lookup_object('print_stats').get_status(0)['state']
        if self.active or self.recovering or (state in ('printing','paused') and (original or not self.armed)):
            raise RequestRefused('rough preparation is not allowed during this job')
        self._invalidate_checkpoint()
        self._invalidate_preparation()
        self._preparing = True
        self._cold_datum = None
        sequence = self._optical_sequence
        success = False
        try:
            self._flags(True)
            self.za.zdown_switch_enable = 1
            home()
            self._settle()
            if (self._optical_sequence != sequence + 1 or self._optical_reference is None
                    or not self.za.is_already_zodwn or self._cold_datum is None):
                raise ValueError('no complete single optical baseline from rough preparation')
            self.preparation_ready = True
            logging.info('KAMP 2.4.1: optical preparation complete; generation=%s', self._optical_sequence)
            success = True
        finally:
            self._preparing = False
            self._flags(False)
            if not success:
                self._invalidate_preparation()

    def cmd_HOT_PREPARE(self, gcmd):
        self._require_ready(gcmd)
        if (not self.armed or self.active or not self.preparation_ready
                or self._optical_reference is None or not self.za.is_already_zodwn):
            raise ValueError('no valid optical preparation; restart the print preparation')
        reference = self._optical_reference
        sequence = self._optical_sequence
        self.preparation_ready = False  # One-use mechanical authorization.
        self._hot_running = True
        self._pending_adjustment = None
        try:
            self.gcode.run_script_from_command('BED_MESH_CLEAR')
            self._flags(True)
            self.gcode.run_script_from_command('Z_TILT_ADJUST WAITTIME=5')
            self._settle()
            if not self.zt.z_status.applied or self._pending_adjustment is None:
                raise ValueError('hot Z tilt did not complete')
            adjustment = self._pending_adjustment
            self._flags(False)
            self.gcode.run_script_from_command('NEXT_HOMEZ_NACCU\nG28 Z')
            final = self._z_positions()
            if self._optical_sequence != sequence or self._optical_reference is not reference:
                raise ValueError('optical baseline changed during hot preparation')
            z = self._toolhead().get_position()[2]
            datums = []
            for name in ('stepper_z', 'stepper_z1'):
                p0, step = reference[name]
                p1, final_step = final[name]
                if step != final_step or not 0 < step < .1 or p1 >= p0:
                    raise ValueError('invalid/changed Z step counter reference')
                b = adjustment[name+'_adjustment']
                datums.append(-(p1-p0)*step + b + z)
            # First Z is the stock datum source. Second Z checks consistency;
            # allowance is eight motor steps for endpoint quantization/rounding.
            tolerance = 8 * max(value[1] for value in reference.values())
            if abs(datums[0]-datums[1]) > tolerance:
                raise ValueError('dual-Z hot datum disagreement')
            mechanics = {'datum': {'zmax': datums[0]}, 'adjustment': adjustment}
            self._validate_mechanics(mechanics)
            self._publish_mechanics(mechanics)
            self._mechanical_generation = mechanics
            logging.info('KAMP 2.4.1: hot preparation complete; mechanics=%s', mechanics)
        finally:
            self._hot_running = False
            self._flags(False)
            self._optical_reference = None

    def _validate_mechanics(self, value):
        zmax = value['datum']['zmax']
        maximum = self.printer.lookup_object('configfile').get_status(0)['settings']['stepper_z']['position_max']
        if not math.isfinite(zmax) or not maximum-15 <= zmax <= maximum:
            raise ValueError('hot Z datum outside reviewed range')
        if set(value['adjustment']) != {'stepper_z_adjustment','stepper_z1_adjustment'}:
            raise ValueError('invalid dual-Z adjustment record')
        if not all(math.isfinite(v) and abs(v) < 10 for v in value['adjustment'].values()):
            raise ValueError('invalid dual-Z adjustment magnitude')

    def _mechanics(self):
        return {'datum':read_json(self.za.real_zmax_path),
                'adjustment':read_json(self.zt.stepper_adjustment_path)}

    def _publish_mechanics(self, value):
        self._validate_mechanics(value)
        # Write-ahead journal makes a power loss between two replaces repairable
        # BEFORE any next homing is accepted after boot.
        atomic_json(self.transaction_path, envelope(value))
        atomic_json(self.zt.stepper_adjustment_path, value['adjustment'])
        atomic_json(self.za.real_zmax_path, value['datum'])
        os.unlink(self.transaction_path)
        sync_dir(self.state_dir)

    def _repair_mechanics(self):
        if os.path.exists(self.transaction_path):
            value = checked_envelope(self.transaction_path)
            self._publish_mechanics(value)

    def _fingerprint(self):
        config = self.printer.lookup_object('configfile').get_status(0)['config']
        sources = {}
        for path in EXPECTED_SOURCES:
            with open(path,'rb') as stream:
                sources[path] = digest(stream.read())
        # Include the adapter, macros, and interpolation configuration as loaded.
        for name in ('kamp_recovery.py','restore_bed_mesh.py'):
            path = os.path.join(os.path.dirname(__file__),name)
            with open(path,'rb') as stream:
                sources[path] = digest(stream.read())
        return digest(canonical({'config':config,'sources':sources}))

    def _identity(self, path):
        root = os.path.realpath(self.sd.sdcard_dirname)
        path = os.path.realpath(path)
        if os.path.commonpath([root,path]) != root or not os.path.isfile(path):
            raise ValueError('print file is outside the configured G-code directory')
        before = os.stat(path)
        hasher = hashlib.sha256()
        with open(path,'rb') as stream:
            while True:
                block = stream.read(1024*1024)
                if not block:break
                hasher.update(block)
        after = os.stat(path)
        if (before.st_size,before.st_mtime_ns) != (after.st_size,after.st_mtime_ns):
            raise ValueError('print file changed during validation')
        return {'path':os.path.relpath(path,root),'size':after.st_size,'sha256':hasher.hexdigest()}

    def write_job_record(self, path, data):
        # Stock's periodic writer otherwise discards unknown keys and writes
        # in place. Include the job generation on EVERY atomic replacement.
        if self.job_id is not None:
            data['kamp_job_id'] = self.job_id
        atomic_json(path,data)

    def _save_checkpoint(self):
        if not self.job_id or self._mechanical_generation is None:
            raise ValueError('no completed mechanical calibration for checkpoint')
        if self._mechanics() != self._mechanical_generation:
            raise ValueError('mechanical calibration changed before mesh checkpoint')
        mesh = self.session_mesh
        if not mesh.isenable or mesh.z_temp_compensation_callbak is not None:
            raise ValueError('unreviewed mesh enable/temperature callback for recovery')
        fd_stat = os.fstat(self.sd.current_file.fileno())
        path_stat = os.stat(self.sd.current_file.name)
        if not os.path.samestat(fd_stat,path_stat):
            raise ValueError('open print descriptor differs from the file on disk')
        # Also keep an atomic job record when stock power-loss is disabled;
        # normal printing remains available, but recovery still requires EEPROM.
        self.gm.recordPrintFileName(self.sd.print_file_name_path,self.sd.current_file.name)
        record = read_json(self.sd.print_file_name_path)
        if record.get('kamp_job_id') != self.job_id:
            raise ValueError('stock recovery metadata has no matching job generation')
        identity=self._identity(self.sd.current_file.name)
        if identity != self._selected_job or not os.path.samestat(
                os.fstat(self.sd.current_file.fileno()),os.stat(self.sd.current_file.name)):
            raise ValueError('selected print file changed before checkpoint')
        value = {'schema':1,'job_id':self.job_id,
                 'job':identity,
                 'fingerprint':self._fingerprint(),
                 'minimum_position':int(self.sd.next_file_position),
                 'mechanics':self._mechanical_generation,
                 'mesh':{'params':dict(mesh.get_mesh_params()),
                         'points':mesh.get_probed_matrix(),
                         'offsets':list(mesh.mesh_offsets)}}
        self._validate_mesh(value['mesh'])
        atomic_json(self.checkpoint_path,envelope(value))

    def _disable_eeprom(self, chip):
        chip.setEepromDisable()
        if not chip.checkEepromFirstEnable():
            raise ValueError('could not invalidate old EEPROM recovery record')

    def commit_eeprom(self, chip, position, base_e):
        if not self.active or self.job_id is None:
            raise ValueError('EEPROM update has no active KAMP generation')
        value = checked_envelope(self.checkpoint_path)
        if value['job_id'] != self.job_id:
            raise ValueError('EEPROM/snapshot generation mismatch')
        self._verify_position(value,{'file_position':position,'base_position_e':base_e})
        # Disable/read back BEFORE touching a body; enable LAST, only after a
        # durable generation record and matching body/header read back.
        self._disable_eeprom(chip)
        slot = chip.eepromReadHeader() % 255 + 1
        chip.write_reg(slot*8,list(int(position).to_bytes(4,'little')))
        chip.write_reg(slot*8+4,list(struct.pack('f',base_e)))
        chip.write_reg(0,slot)
        expected={'file_position':position,'base_position_e':struct.unpack('f',struct.pack('f',base_e))[0]}
        if chip.eepromReadHeader()!=slot or chip.eepromReadBody(slot)!=expected:
            raise ValueError('EEPROM body/header readback failed')
        atomic_json(self.eeprom_path,envelope({'job_id':self.job_id,'slot':slot,'info':expected}))
        chip.write_reg(1,1)
        if chip.checkEepromFirstEnable():
            raise ValueError('EEPROM enable readback failed')

    def _validate_mesh(self, data):
        p = data['params'];points=data['points']
        for key in ('x_count','y_count'):
            if type(p[key]) is not int or not 3 <= p[key] <= 9:
                raise ValueError('invalid recovered mesh dimensions')
        for key in ('mesh_x_pps','mesh_y_pps'):
            if type(p[key]) is not int or not 0 <= p[key] <= 6:
                raise ValueError('invalid recovered interpolation size')
        if p['algo'] not in ('lagrange','bicubic','direct'):
            raise ValueError('invalid recovered interpolation algorithm')
        for axis in ('x','y'):
            if not 5 <= p['min_'+axis] < p['max_'+axis] <= 345:
                raise ValueError('invalid recovered mesh bounds')
        if len(points)!=p['y_count'] or any(len(row)!=p['x_count'] for row in points):
            raise ValueError('recovered mesh point count mismatch')
        if not all(math.isfinite(v) and abs(v)<10 for row in points for v in row):
            raise ValueError('invalid recovered mesh height')
        if len(data['offsets'])!=2 or any(v != 0 for v in data['offsets']):
            raise ValueError('only zero mesh offsets are supported for recovery')
        if not math.isfinite(p['tension']) or not 0 <= p['tension'] <= 2:
            raise ValueError('invalid mesh interpolation tension')

    def _verify_checkpoint(self, path):
        value = checked_envelope(self.checkpoint_path)
        if value['schema'] != 1 or value['job'] != self._identity(path):
            raise ValueError('recovery snapshot belongs to another/changed print file')
        if value['fingerprint'] != self._fingerprint():
            raise ValueError('firmware or loaded configuration changed since checkpoint')
        self._validate_mechanics(value['mechanics'])
        if value['mechanics'] != self._mechanics():
            raise ValueError('Z calibration changed since the interrupted print')
        self._validate_mesh(value['mesh'])
        record = read_json(self.sd.print_file_name_path)
        if record.get('kamp_job_id') != value['job_id'] or os.path.realpath(record['file_path']) != os.path.realpath(path):
            raise ValueError('stock checkpoint belongs to another job generation')
        enabled = self.printer.lookup_object('bl24c16f')
        if enabled.checkEepromFirstEnable():
            raise ValueError('no enabled stock EEPROM checkpoint')
        info = enabled.eepromReadBody(enabled.eepromReadHeader())
        generation=checked_envelope(self.eeprom_path)
        if (generation['job_id']!=value['job_id'] or generation['slot']!=enabled.eepromReadHeader()
                or generation['info']!=info):
            raise ValueError('EEPROM checkpoint does not match its durable job generation')
        self._verify_position(value,info)
        return value

    def _verify_position(self, value, info):
        position = info.get('file_position')
        if (type(position) is not int or not value['minimum_position'] <= position < value['job']['size']
                or not math.isfinite(info.get('base_position_e',float('nan')))):
            raise ValueError('invalid stock EEPROM recovery offset')

    def cmd_START_FILE(self, gcmd):
        self._require_ready(gcmd)
        state = self.printer.lookup_object('print_stats').get_status(0)['state']
        if self.sd.is_active() or state in ('printing','paused'):
            raise RequestRefused('cancel the previous job before selecting a file')
        if 'ISCONTINUEPRINT' in gcmd.get_command_parameters():
            if not gcmd.get('ISCONTINUEPRINT'):
                raise RequestRefused('empty ISCONTINUEPRINT is ambiguous; supply 1')
            # Creality interprets any NONEMPTY supplied string as true. Match its actual
            # semantics rather than treating ISCONTINUEPRINT=0 as a fresh job.
            prefs = read_json(self.sd.user_print_refer_path)
            if not prefs.get('power_loss',{}).get('switch',False):
                raise ValueError('stock power-loss recovery is disabled')
            filename = gcmd.get('FILENAME','').lstrip('/')
            path = os.path.join(self.sd.sdcard_dirname,filename)
            value = self._verify_checkpoint(path)  # Before stock can heat/move.
            from extras.bed_mesh import ZMesh
            mesh = ZMesh(value['mesh']['params'],self.printer)
            mesh.build_mesh(value['mesh']['points'])
            mesh.set_mesh_offsets(value['mesh']['offsets'])
            self.session_mesh = mesh
            self.active = False
            self.recovering = True
            self.recovery_optical_done = False
            self.job_id = value['job_id']
            self._recovery_data = value
            self.faulted = False
            self._flags(False)
        else:
            self.cmd_PREPARE_FILE(gcmd)
            self._invalidate_checkpoint()
            chip=self.printer.lookup_object('bl24c16f',None)
            if chip is not None:self._disable_eeprom(chip)
            self.job_id = uuid.uuid4().hex
            filename=gcmd.get('FILENAME','').lstrip('/')
            self._selected_job=self._identity(os.path.join(self.sd.sdcard_dirname,filename))
        self.gcode.run_script_from_command('_KAMP_SDCARD_BASE '+gcmd.get_raw_command_parameters())
        if self.sd.current_file is None or not self.sd.is_active():
            raise ValueError('stock SD start did not accept the file')

    def _restore_stock(self, print_info, path, xyzet):
        try:
            if not self.recovering or self._recovery_data is None:
                raise ValueError('stock recovery has no verified KAMP snapshot')
            # Revalidate the checkpoint selected by the asynchronous SD worker.
            value = self._verify_checkpoint(self.sd.current_file.name)
            self._verify_position(value,print_info)
            if (not isinstance(xyzet,dict)
                    or any(not math.isfinite(xyzet.get(k,float('nan'))) for k in ('X','Y','Z','E'))
                    or not 0 < xyzet['Z'] < 360 or self.sd.restore_err):
                raise ValueError('incomplete stock recovery coordinates')
            box = self.printer.lookup_object('box',None)
            if box is not None:
                box_state = read_json(box.box_state.tn_save_data_path)
                if box_state.get('enable') not in (0,1):
                    raise ValueError('invalid stock CFS recovery state')
            # Restore mechanical calibration at the original bed temperature.
            bed = self.sd.get_print_temperature(self.sd.current_file.name)[0]
            self.gcode.run_script('M190 S%s' % bed)
            self.check_recovery_phase()
            self._flags(False)
            result = self._stock_restore(print_info,path,xyzet)
            if (result is not True or not self.recovery_optical_done or self.faulted
                    or self.sd.must_pause_work or self.bedmesh.get_mesh() is not self.session_mesh):
                raise ValueError('stock recovery did not complete successfully')
            self.active = True
            self.recovering = False
        except Exception:
            logging.exception('KAMP stock recovery did not complete')
            self.sd.must_pause_work = True
            self.sd.restore_err = True
            self.sd.restore_print_timer = None
            self._safe_stop()
            # The stock restore_print callback tests must_pause_work immediately
            # after this method. Do not leak exceptions out of its reactor timer.
            self.sd.must_pause_work = True
            self.sd.restore_err = True
            self.sd.restore_print_timer = None
            return False

    def check_recovery_phase(self):
        self.gcode.check_cancel_running()
        if (not self.recovering or self._recovery_data is None or self.faulted
                or self.sd.must_pause_work or self.job_id != self._recovery_data['job_id']):
            raise ValueError('recovery was cancelled or invalidated')
