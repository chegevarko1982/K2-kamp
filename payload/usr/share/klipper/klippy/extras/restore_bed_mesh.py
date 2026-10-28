"""K2 Plus mesh-only adapter. Derived from KAMP-K2 (GPL-3.0).

Dedicated command; no replacement of Creality's calibration handlers.
Runtime compatibility checks deliberately reject unverified callback layouts.
"""
import logging
import math
from extras.kamp_recovery import Recovery, RequestRefused


class CommandRefused(Exception):
    """A rejected external request must not disturb a valid running job."""


class BedMeshOverride(Recovery):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.ready = False
        self.error = 'initialization not complete'
        self.active = False
        self.armed = False
        self.session_mesh = None
        self.bedmesh = None
        self.upstream_cmd = None
        self.layout = {}
        self.faulted = False
        self.manual_busy = False
        self.full_armed = False
        self.probing = False
        self._configure_recovery(config)
        commands = {
            'KAMP_VALIDATE_START': self.cmd_VALIDATE_START,
            'KAMP_VALIDATE_FULL': self.cmd_VALIDATE_FULL,
            'KAMP_FULL_BED_CALIBRATE': self.cmd_FULL_CALIBRATE,
            'KAMP_PREPARE_FILE': self.cmd_PREPARE_FILE,
            'KAMP_CHECK_RESUME': self.cmd_CHECK_RESUME,
            '_KAMP_PROBE': self.cmd_PROBE,
            'KAMP_SESSION_CLEAR': self.cmd_SESSION_CLEAR,
            'KAMP_RESTORE_SESSION': self.cmd_RESTORE_SESSION,
            'KAMP_ABORT': self.cmd_ABORT,
            '_KAMP_REFUSE_PROFILE': self.cmd_REFUSE_PROFILE,
            '_KAMP_REJECT_RECOVERY': self.cmd_REJECT_RECOVERY,
        }
        # Duplicate/failed registrations propagate as a config error.
        for name, callback in commands.items():
            self.gcode.register_command(name, self._guard(callback))
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

    def _guard(self, callback):
        def run(gcmd):
            try:
                return callback(gcmd)
            except (CommandRefused, RequestRefused) as exc:
                raise gcmd.error('KAMP: ' + str(exc)) from exc
            except Exception as exc:
                self._safe_stop()
                raise gcmd.error('KAMP: ' + str(exc)) from exc
        return run

    def _handle_ready(self):
        try:
            from extras.bed_mesh import BedMeshCalibrate
            from extras.probe import ProbePointsHelper
            self.bedmesh = self.printer.lookup_object('bed_mesh')
            bmc = self.bedmesh.bmc
            # cmd_BED_MESH_CALIBRATE uses dynamic methods and an existing
            # probe-helper callback. Only the inspected upstream path is allowed.
            for name in ('update_config', 'probe_finalize', '_verify_algorithm',
                         '_generate_points', '_get_adjusted_points', 'print_generated_points'):
                method = getattr(bmc, name, None)
                self.layout[name] = repr(method)
                if getattr(method, '__func__', None) is not getattr(BedMeshCalibrate, name):
                    raise RuntimeError('unverified calibration method: ' + name)
            start_probe = getattr(bmc.probe_helper, 'start_probe', None)
            self.layout['start_probe'] = repr(start_probe)
            if getattr(start_probe, '__func__', None) is not ProbePointsHelper.start_probe:
                raise RuntimeError('unverified probe-helper start_probe implementation')
            callback = getattr(bmc.probe_helper, 'finalize_callback', None)
            self.layout['finalize_callback'] = repr(callback)
            if (getattr(callback, '__func__', None) is not BedMeshCalibrate.probe_finalize
                    or getattr(callback, '__self__', None) is not bmc):
                raise RuntimeError('unverified probe-helper completion callback')
            if not callable(getattr(self.bedmesh, 'save_profile', None)):
                raise RuntimeError('bed_mesh.save_profile unavailable')
            # This exact registry/API was read from this printer's webhooks.py.
            # Replacing the instance method alone would NOT replace the bound
            # callback already stored in the endpoint registry.
            from extras.bed_mesh import BedMesh
            webhooks = self.printer.lookup_object('webhooks')
            endpoints = getattr(webhooks, '_endpoints', None)
            endpoint = endpoints.get('update_mesh') if isinstance(endpoints, dict) else None
            if (getattr(endpoint, '__self__', None) is not self.bedmesh
                    or getattr(endpoint, '__func__', None) is not BedMesh.update_mesh):
                raise RuntimeError('unverified update_mesh webhook registration')
            self.original_update_mesh = endpoint
            endpoints['update_mesh'] = self._guard_update_mesh
            if endpoints['update_mesh'] != self._guard_update_mesh:
                raise RuntimeError('update_mesh webhook guard registration failed')
            self.upstream_cmd = BedMeshCalibrate.cmd_BED_MESH_CALIBRATE.__get__(bmc, type(bmc))
            self._init_recovery()
            self.ready = True
            self.error = ''
            logging.info('KAMP v2: dedicated _KAMP_PROBE ready; layout=%s', self.layout)
        except Exception as exc:
            self.error = str(exc)
            self.ready = False
            logging.exception('KAMP v2 initialization failed')
            # Do not leave Klipper ready after an adapter initialization failure.
            self.printer.invoke_shutdown('KAMP initialization failed: ' + self.error)

    def get_status(self, eventtime):
        return {'ready': self.ready, 'error': self.error, 'active': self.active,
                'armed': self.armed, 'faulted': self.faulted, 'manual_busy': self.manual_busy, 'probing': self.probing,
                'session_only': False, 'recovering': self.recovering, 'preparation_ready': self.preparation_ready, 'layout': dict(self.layout)}

    def _guard_update_mesh(self, request):
        # Refuse before mutating the matrix or invoking any save/load/CXSAVE.
        if not self.ready or self.active or self.recovering or self.armed or self.manual_busy or self.probing or self.faulted:
            raise request.error('KAMP: mesh editing is blocked during this session')
        return self.original_update_mesh(request)

    def _clear(self):
        self.armed = False
        self.active = False
        self.session_mesh = None
        if self.bedmesh is not None:
            self.bedmesh.set_mesh(None)

    def _safe_stop(self):
        # No G28, parking, extrusion, motor disable, CANCEL_PRINT or CFS move.
        self.faulted = True
        self.full_armed = False
        try:
            sd = self.printer.lookup_object('virtual_sdcard')
            sd.do_pause()  # Stops new file commands; it does not park the head.
            if sd.is_active() and not sd.must_pause_work:
                raise RuntimeError('SD input did not accept the stop request')
        except Exception:
            logging.exception('KAMP could not stop SD command input')
            self.printer.invoke_shutdown('KAMP could not stop SD command input')
            return  # Do not clear compensation while input might still run.
        try:
            self.printer.lookup_object('heaters').turn_off_all_heaters()
        except Exception:
            logging.exception('KAMP could not turn off heaters')
            self.printer.invoke_shutdown('KAMP could not turn off heaters')
        try:
            self._invalidate_checkpoint()
            self._invalidate_preparation()
            self._clear()
        except Exception:
            logging.exception('KAMP could not clear mesh state')
            self.printer.invoke_shutdown('KAMP mesh cleanup failed')
        for command in ('PRINT_PREPARE_CLEAR', 'EXCLUDE_OBJECT_RESET', 'SET_G29_FLAG VALUE=0'):
            try:
                self.gcode.run_script_from_command(command)
            except Exception:
                logging.exception('KAMP error cleanup failed: %s', command)
                self.printer.invoke_shutdown('KAMP preparation cleanup failed')

    def _require_ready(self, gcmd):
        if not self.ready or self.upstream_cmd is None:
            raise gcmd.error('KAMP adapter is not ready: ' + self.error)

    def _objects(self):
        objects = self.printer.lookup_object('exclude_object').get_status(0)['objects']
        points = []
        if not objects:
            raise ValueError('no object boundaries; enable EXCLUDE_OBJECT_DEFINE before START_PRINT')
        for obj in objects:
            polygon = obj.get('polygon', [])
            if len(polygon) < 3:
                raise ValueError('an object has no usable polygon')
            for point in polygon:
                if len(point) != 2 or not all(math.isfinite(float(v)) for v in point):
                    raise ValueError('invalid object coordinates')
                points.append(tuple(float(v) for v in point))
        # Objects reaching past the configured mesh window are clamped to it: the
        # adaptive macro already clamps the probed area to mesh_min/mesh_max, and
        # _KAMP_PROBE below still requires 5..345 mm, so no point is probed outside.
        points = [(min(max(x, 5.0), 345.0), min(max(y, 5.0), 345.0)) for x, y in points]
        return points

    def cmd_VALIDATE_START(self, gcmd):
        self._require_ready(gcmd)
        if gcmd.get_int('MESH', 1) != 1:
            raise gcmd.error('KAMP v2 requires MESH=1; mesh skipping is unsupported')
        if self.active or self.armed:
            raise CommandRefused('KAMP session already active; cancel the previous job first')
        status = self.printer.lookup_object('print_stats').get_status(0)
        if status['state'] != 'printing':
            raise gcmd.error('KAMP START_PRINT must execute from a print file')
        self._objects()
        prepared = self.printer.lookup_object('gcode_macro START_PRINT').get_status(0)['prepare']
        if int(prepared) == 1 and not self.preparation_ready:
            raise gcmd.error('LAN preparation has no valid KAMP optical baseline; restart preparation')
        self.armed = True

    def cmd_PROBE(self, gcmd):
        self._require_ready(gcmd)
        mode = gcmd.get('MODE', '').upper()
        if mode not in ('ADAPTIVE', 'FULL'):
            raise CommandRefused('KAMP requires MODE=ADAPTIVE or FULL')
        if mode == 'ADAPTIVE':
            if not self.armed or self.active:
                raise CommandRefused('KAMP adaptive probe has no one-use START_PRINT authorization')
            self._objects()
            self.armed = False  # Consume before any probe can run, including failures.
        elif (not self.manual_busy or not self.full_armed or self.active or self.armed
              or self.printer.lookup_object('print_stats').get_status(0)['state'] in ('printing', 'paused')):
            raise CommandRefused('KAMP full calibration requires the idle manual sequence')
        else:
            self.full_armed = False
        bounds = []
        for name in ('MESH_MIN', 'MESH_MAX'):
            pair = tuple(float(v) for v in gcmd.get(name, '').split(','))
            if len(pair) != 2 or not all(math.isfinite(v) and 5 <= v <= 345 for v in pair):
                raise gcmd.error('invalid ' + name)
            bounds.append(pair)
        if any(bounds[0][i] >= bounds[1][i] for i in (0, 1)):
            raise gcmd.error('invalid mesh bounds ordering')
        profile = '_kamp_session' if mode == 'ADAPTIVE' else 'default'
        if gcmd.get('PROFILE', '') != profile:
            raise gcmd.error('incorrect KAMP profile for mode ' + mode)
        if mode == 'FULL' and bounds != [(5.0, 5.0), (345.0, 345.0)]:
            raise gcmd.error('full calibration must cover the complete configured bed')
        original_save = self.bedmesh.save_profile
        save_called = []
        if mode == 'ADAPTIVE':
            def session_save(name):
                if name != '_kamp_session':
                    raise gcmd.error('unexpected persistent profile during adaptive probe')
                save_called.append(name)  # Never call configfile.set or mutate default.
            self.bedmesh.save_profile = session_save
        try:
            self.probing = True
            self.upstream_cmd(gcmd)
            if mode == 'ADAPTIVE':
                if save_called != ['_kamp_session'] or self.bedmesh.get_mesh() is None:
                    raise gcmd.error('unverified or incomplete adaptive probe completion')
                self.session_mesh = self.bedmesh.get_mesh()
                self.active = True
                self._save_checkpoint()
        finally:
            self.probing = False
            self.bedmesh.save_profile = original_save

    def cmd_VALIDATE_FULL(self, gcmd):
        self._require_ready(gcmd)
        state = self.printer.lookup_object('print_stats').get_status(0)['state']
        if self.active or self.armed or state in ('printing', 'paused'):
            raise CommandRefused('full calibration is only allowed with no active print')

    def cmd_FULL_CALIBRATE(self, gcmd):
        self.cmd_VALIDATE_FULL(gcmd)
        self._invalidate_checkpoint()
        self._invalidate_preparation()
        sensor = self.printer.lookup_object('filament_switch_sensor filament_sensor')
        enabled = bool(sensor.get_status(0)['enabled'])
        self.manual_busy = True
        self.full_armed = True
        try:
            # run_script_from_command propagates errors from EVERY nested
            # command, not just the later probe. The outer guard handles them.
            self.gcode.run_script_from_command('_KAMP_FULL_SEQUENCE ' + gcmd.get_raw_command_parameters())
        finally:
            self.manual_busy = False
            self.full_armed = False
            try:
                self.gcode.run_script_from_command('SET_FILAMENT_SENSOR SENSOR=filament_sensor ENABLE=%d' % enabled)
            except Exception:
                self.printer.invoke_shutdown('KAMP could not restore the filament sensor')
                raise

    def cmd_PREPARE_FILE(self, gcmd):
        sd = self.printer.lookup_object('virtual_sdcard')
        state = self.printer.lookup_object('print_stats').get_status(0)['state']
        if sd.is_active() or state in ('printing', 'paused'):
            raise CommandRefused('cancel the previous job before starting a new file')
        self._clear()
        self.faulted = False

    def cmd_CHECK_RESUME(self, gcmd):
        if self.faulted:
            raise CommandRefused('this job was aborted; cancel it and start a new file, do not resume')

    def cmd_SESSION_CLEAR(self, gcmd):
        self._invalidate_checkpoint()
        self._invalidate_preparation()
        self._clear()

    def cmd_RESTORE_SESSION(self, gcmd):
        self._require_ready(gcmd)
        if self.recovering and not self.recovery_optical_done:
            self.bedmesh.set_mesh(None)
            return
        if not (self.active or self.recovering) or self.session_mesh is None:
            raise gcmd.error('no active KAMP mesh to restore')
        self.bedmesh.set_mesh(self.session_mesh)

    def cmd_ABORT(self, gcmd):
        self._safe_stop()

    def cmd_REFUSE_PROFILE(self, gcmd):
        raise CommandRefused('KAMP session mesh is temporary; profile changes are blocked during the job')

    def cmd_REJECT_RECOVERY(self, gcmd):
        raise CommandRefused('KAMP session mesh is not persisted; ISCONTINUEPRINT is unsupported')


def load_config(config):
    return BedMeshOverride(config)
