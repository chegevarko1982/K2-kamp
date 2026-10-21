"""Offline 2.4 integration: real firmware methods, fake hardware and EEPROM."""
import ast
import configparser
import contextlib
import importlib.util
import json
import logging
import math
import os
import struct
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'tests/results'
OUT.mkdir(parents=True,exist_ok=True)
PAYLOAD=ROOT/'payload'
sys.dont_write_bytecode=True
from jinja2 import Environment
logging.disable(logging.CRITICAL)
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec);sys.modules[name]=mod;spec.loader.exec_module(mod);return mod
sys.modules['extras']=types.ModuleType('extras');sys.modules['extras'].__path__=[]
r=load('extras.kamp_recovery',PAYLOAD/'usr/share/klipper/klippy/extras/kamp_recovery.py')
# Directory fsync is exercised on the Linux device during preflight; Windows
# cannot open a directory with os.open. File fsync/replace remain real here.
r.sync_dir=lambda path:None
a=load('adapter24',PAYLOAD/'usr/share/klipper/klippy/extras/restore_bed_mesh.py')

def method(path,cls,name):
    tree=ast.parse(path.read_text(encoding='utf-8'))
    c=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls)
    m=next(n for n in c.body if isinstance(n,ast.FunctionDef) and n.name==name)
    return m
def compile_method(node,extra=None):
    ns={'logging':logging,'os':os,'math':math};ns.update(extra or {})
    exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_reviewed_firmware','exec'),ns)
    return ns[node.name]
SRC=ROOT/'tests/fixtures/klipper'
ZADJUST=compile_method(method(SRC/'extras/z_tilt.py','ZAdjustHelper','adjust_steppers'))
GM=PAYLOAD/'usr/share/klipper/klippy/extras/gcode_move.py'
RESTORE=compile_method(method(GM,'GCodeMove','cmd_CX_RESTORE_GCODE_STATE'))
RECORD=compile_method(method(GM,'GCodeMove','recordPrintFileName'))
class HomingState:
    def __init__(self,p):self.out_z_all=0
    def set_axes(self,axes):self.axes=axes
HOME=compile_method(method(SRC/'extras/homing.py','PrinterHoming','cmd_G28'),{'Homing':HomingState,'MOTOR_PROTECT_ERROR':-10001})

class Command:
    def __init__(self,**kw):self.params={k.upper():str(v) for k,v in kw.items()}
    def get(self,k,default=None):return self.params.get(k,default)
    def get_int(self,k,default=None,**kw):return int(self.get(k,default))
    def get_command_parameters(self):return self.params
    def get_raw_command_parameters(self):return ' '.join(k+'='+v for k,v in self.params.items())
    def error(self,msg):return ValueError(msg)
class Mesh:
    def __init__(self,params,printer=None):
        self.params=params;self.mesh_offsets=[0,0];self.isenable=True;self.z_temp_compensation_callbak=None
    def build_mesh(self,m):self.matrix=m
    def get_mesh_params(self):return self.params
    def get_probed_matrix(self):return self.matrix
    def set_mesh_offsets(self,o):self.mesh_offsets=list(o)
sys.modules['extras.bed_mesh']=types.SimpleNamespace(ZMesh=Mesh)
class Stepper:
    def __init__(self,name):self.name=name;self.position=0;self.step=.0025;self.enabled=True
    def get_name(self):return self.name
    def get_mcu_position(self):return round(self.position/self.step)
    def get_step_dist(self):return self.step
    def _query_mcu_position(self):pass
    def set_trapq(self,q):self.enabled=q is not None
class Bed:
    def __init__(self):self.mesh=None
    def get_mesh(self):return self.mesh
    def set_mesh(self,m):self.mesh=m
class Engine(a.BedMeshOverride):
    instances=[]
    eeprom_states={}
    def __init__(self,root):
        self.instances.append(self)
        # Hardware mocks; all tested preparation/persistence/recovery code below
        # is the actual candidate, not a rewritten state machine.
        self.root=Path(root);self.events=[];self.commands={};self.failure=None;self.optical_count=0
        self.ready=True;self.error='';self.active=self.armed=self.faulted=self.manual_busy=self.full_armed=self.probing=False
        self.session_mesh=None;self.upstream_cmd=lambda cmd:None
        self.fingerprint='reviewed-config';self.state='standby';self.off=0
        self.steppers=[Stepper('stepper_z'),Stepper('stepper_z1')]
        self.position=[175.,175.,5.,0.];self.hot_datum=352.1875;self.cold_datum=352.
        self.skip_tilt=False;self.bad_optical=False
        self.tool=types.SimpleNamespace(G29_flag=False,get_max_accel=lambda:5000,
            get_kinematics=lambda:types.SimpleNamespace(get_steppers=lambda:self.steppers,home=self.home),
            get_status=lambda now:{'homed_axes':'xyz'},get_position=lambda:list(self.position),
            set_position=self.set_position,move=self.move,wait_moves=lambda:None,
            flush_step_generation=lambda:None,get_trapq=lambda:object(),
            set_max_accel=lambda value:None,requested_accel_to_decel=5000,square_corner_velocity=10)
        self.gcode=types.SimpleNamespace(register_command=lambda name,cb:self.commands.update({name:cb}),
            run_script_from_command=self.execute,_respond_error=lambda msg:self.events.append('ERROR '+msg),
            run_script=self.execute,check_cancel_running=lambda:None,
            respond_info=lambda msg:None,get_mutex=lambda:contextlib.nullcontext(),
            exclude_object_info=str(self.root/'exclude.json'))
        self.bedmesh=Bed()
        self.za=types.SimpleNamespace(pin_len=2,is_already_zodwn=False,zdown_switch_enable=0,force_stop_flag=False,
            cur_retries=0,retries=5,retry_tolerance=10,real_zmax_path=str(self.root/'real_zmax.json'),
            mcu=types.SimpleNamespace(_serial=types.SimpleNamespace(z_align_status={'flag':1,'deltaError1':0})))
        self.zt=types.SimpleNamespace(stepper_z_adjustment=0.,stepper_z1_adjustment=0.,
            stepper_adjustment_path=str(self.root/'stepper_adjustment.json'),
            z_status=types.SimpleNamespace(applied=False))
        self.hm=types.SimpleNamespace(write_real_zmax=self._stage_datum)
        self.sd=types.SimpleNamespace(sdcard_dirname=str(self.root/'gcodes'),current_file=None,next_file_position=100,
            file_position=0,must_pause_work=False,restore_err=False,restore_print_timer=None,work_timer=None,
            is_active=lambda:self.sd.work_timer is not None,do_pause=self.pause,
            print_file_name_path=str(self.root/'print_file_name.json'),user_print_refer_path=str(self.root/'prefs.json'),
            offset_value=0.,gcode_metadata=None,get_print_temperature=lambda filename:(100,260,0))
        self.gm=types.SimpleNamespace(printer=None,absolute_coord=True,absolute_extrude=False,
            config=types.SimpleNamespace(has_section=lambda name:name=='z_align'),
            get_status=lambda now:{'position':list(self.position)},
            speed=25.,speed_factor=1/60,extrude_factor=1.,last_position=[175.,175.,5.,0.],
            base_position=[0.,0.,0.,0.],homing_position=[0.,0.,0.,0.],variable_safe_z=3.,
            move_with_transform=self.transformed,power_loss_sensor_detected=lambda:True)
        self.chip_state=self.eeprom_states.setdefault(str(root),{'header':3,'enabled':True,'info':{'file_position':1000,'base_position_e':12.}})
        self.checkpoint_position=self.chip_state['info']
        self.eeprom_enabled=True
        self.eeprom=types.SimpleNamespace(checkEepromFirstEnable=lambda:not(self.eeprom_enabled and self.chip_state['enabled']),
            setEepromDisable=lambda:self.chip_state.update(enabled=False),write_reg=self.write_reg,
            eepromReadHeader=lambda:self.chip_state['header'],eepromReadBody=lambda pos:dict(self.checkpoint_position))
        self.objects={'gcode':self.gcode,'toolhead':self.tool,'z_align':self.za,'z_tilt':self.zt,
            'virtual_sdcard':self.sd,'gcode_move':self.gm,'restore_bed_mesh':self,'bed_mesh':self.bedmesh,
            'homing':self.hm,'bl24c16f':self.eeprom,
            'print_stats':types.SimpleNamespace(get_status=lambda now:{'state':self.state}),
            'heaters':types.SimpleNamespace(turn_off_all_heaters=self.turn_off),
            'configfile':types.SimpleNamespace(get_status=lambda now:{'settings':{'stepper_z':{'position_max':360}},'config':{'test':self.fingerprint}})}
        self.printer=types.SimpleNamespace(lookup_object=lambda name,default=None:self.objects.get(name,default),
            register_event_handler=lambda *x:None,invoke_shutdown=lambda msg:self.events.append('shutdown '+msg),
            get_reactor=lambda:types.SimpleNamespace(monotonic=lambda:1),command_error=ValueError,is_shutdown=lambda:False)
        self.gm.printer=self.printer
        self.gm.recordPrintFileName=types.MethodType(RECORD,self.gm)
        config=types.SimpleNamespace(get=lambda key,default=None:str(self.root/'state') if key=='state_directory' else default)
        self._configure_recovery(config)
        self._stock_zdown=self.stock_zdown
        self.za.cmd_ZDOWN=self._checked_zdown
        self._stock_datum=lambda value:None
        self._stock_restore=types.MethodType(RESTORE,self.gm)
        self._stock_adjustment=lambda:None
        self.driver=types.SimpleNamespace(printer=self.printer,probe_type='',z_move=54,
            config=types.SimpleNamespace(has_section=lambda key:key=='z_align'),
            run_gcmd=lambda command,**kw:self.execute(command),run_G28_two_Z=lambda:self.za.cmd_ZDOWN(Command()),
            write_real_zmax=self._stage_datum)
        self.driver.set_stall_mode=lambda *x:None
        self.objects['stepper_enable']=types.SimpleNamespace(motor_off=lambda:None)
        self.root.mkdir(exist_ok=True)
        (self.root/'gcodes').mkdir(exist_ok=True)
        self.file=self.root/'gcodes/test.gcode'
        if not self.file.exists():self.file.write_text('; layer data\n'*500)
        if not Path(self.za.real_zmax_path).exists():
            r.atomic_json(self.za.real_zmax_path,{'zmax':352.})
            r.atomic_json(self.zt.stepper_adjustment_path,{'stepper_z_adjustment':.5,'stepper_z1_adjustment':-.2})
        r.atomic_json(self.sd.user_print_refer_path,{'power_loss':{'switch':True}})
    def _fingerprint(self):return self.fingerprint
    def write_reg(self,addr,data):
        if addr==0:self.chip_state['header']=data
        elif addr==1:self.chip_state['enabled']=data!=255
        elif addr%8==0:self.checkpoint_position['file_position']=int.from_bytes(bytes(data),'little')
        elif addr%8==4:self.checkpoint_position['base_position_e']=struct.unpack('f',bytes(data))[0]
    def turn_off(self):self.off+=1;self.events.append('heaters off')
    def pause(self):self.sd.must_pause_work=True;self.sd.work_timer=None;self.events.append('SD stopped')
    def set_position(self,p,**kwargs):self.position=list(p)
    def move(self,p,speed):
        delta=p[2]-self.position[2]
        for stepper in self.steppers:
            if stepper.enabled:stepper.position+=delta
        self.position=list(p)
    def adjust(self,values):
        helper=types.SimpleNamespace(printer=self.printer,z_steppers=self.steppers)
        ZADJUST(helper,values,10.)
    def stock_zdown(self,cmd):
        self.optical_count+=1
        self.events.append('optical')
        for stepper in self.steppers:stepper.position=0
        self.position[2]=360.
        self.za.cur_retries=5 if self.bad_optical else 0
        if not self.tool.G29_flag:
            d=r.read_json(self.za.real_zmax_path)['zmax'];self.position[2]=d
            self.move([175,175,d-10,0],10)
            self.zt.stepper_z_adjustment=self.zt.stepper_z1_adjustment=0
            b=r.read_json(self.zt.stepper_adjustment_path)
            self.adjust([b['stepper_z_adjustment'],b['stepper_z1_adjustment']])
        return 0
    def home(self,state):
        b=self._pending_adjustment or {'stepper_z_adjustment':0.,'stepper_z1_adjustment':0.}
        datum=self.hot_datum if self._hot_running else self.cold_datum
        for stepper in self.steppers:
            stepper.position=-(datum-b[stepper.name+'_adjustment'])
        self.position[2]=0
        state.out_z_all=self.cold_datum-54
    def transformed(self,p,speed):
        if self.bedmesh.get_mesh() is not self.session_mesh:raise AssertionError('move without recovered compensation')
        self.events.append('model move')
        self.move(p,speed)
    def execute(self,script):
        for line in script.splitlines():
            self.events.append(line)
            if line==self.failure:raise ValueError('injected '+line)
            if line=='G28' or line.startswith('_KAMP_ROUGH_BASE') or line=='G28 Z':
                HOME(self.driver,Command(Z=0))
                p=list(self.position);p[2]=5;self.move(p,10)
            elif line.startswith('Z_TILT_ADJUST'):
                self.zt.stepper_z_adjustment=self.zt.stepper_z1_adjustment=0.
                self.adjust([.20,-.15]);self.adjust([.05,.05])
                self.zt.z_status.applied=not self.skip_tilt
                if not self.skip_tilt:self._stage_adjustment()
            elif line=='ZDOWN':self.za.cmd_ZDOWN(Command())
            elif line.startswith('BED_MESH_PROFILE'):
                self.cmd_RESTORE_SESSION(Command())
            elif line=='BED_MESH_CLEAR':self.bedmesh.set_mesh(None)
            elif line.startswith('_KAMP_SDCARD_BASE'):
                if self.sd.current_file is not None:self.sd.current_file.close()
                self.sd.current_file=open(self.file,encoding='utf-8')
                self.sd.work_timer=object()
                if not self.recovering:self.gm.recordPrintFileName(self.sd.print_file_name_path,str(self.file))
    def start_and_checkpoint(self):
        self.cmd_ROUGH_HOME(Command(ORIGINAL=1))
        self.cmd_START_FILE(Command(FILENAME='test.gcode'))
        self.armed=True
        self.cmd_HOT_PREPARE(Command())
        params={'min_x':10.,'max_x':60.,'min_y':10.,'max_y':60.,'x_count':3,'y_count':3,
                'mesh_x_pps':2,'mesh_y_pps':2,'algo':'lagrange','tension':.2}
        self.session_mesh=Mesh(params);self.session_mesh.build_mesh([[.1,.2,.1],[.1,.3,.1],[.1,.2,.1]])
        self.bedmesh.set_mesh(self.session_mesh);self.active=True
        self._save_checkpoint()
        self.commit_eeprom(self.eeprom,1000,12.)
    def restore(self):
        self.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
        return self._restore_stock(dict(self.checkpoint_position),self.sd.print_file_name_path,
                 {'X':50.,'Y':50.,'Z':10.,'E':123.,'T':'','M':''})

class Cases(unittest.TestCase):
    def setUp(self):
        parent=OUT/'tmp';parent.mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=parent);self.root=Path(self.temp.name);self.e=Engine(self.root)
    def tearDown(self):
        for engine in Engine.instances:
            if engine.sd.current_file is not None:engine.sd.current_file.close()
        Engine.instances.clear();self.temp.cleanup()
    def prepared(self):self.e.start_and_checkpoint();return Engine(self.root)
    def test_one_optical_and_hot_datum_not_cold_datum(self):
        self.e.start_and_checkpoint()
        self.assertEqual(self.e.optical_count,1)
        m=self.e._mechanics();self.assertAlmostEqual(m['datum']['zmax'],352.1875,places=5)
        self.assertAlmostEqual(m['adjustment']['stepper_z_adjustment'],.25)
        self.assertAlmostEqual(m['adjustment']['stepper_z1_adjustment'],-.10)
        self.assertFalse(self.e.tool.G29_flag);self.assertEqual(self.e.za.zdown_switch_enable,0)
    def test_direct_start_same_single_optical(self):
        self.e.armed=True;self.e.state='printing';self.e.cmd_ROUGH_HOME(Command())
        self.e.cmd_HOT_PREPARE(Command());self.assertEqual(self.e.optical_count,1)
    def test_repeated_print_has_exactly_one_optical_per_preparation(self):
        self.e.start_and_checkpoint();self.e.sd.work_timer=None;self.e.cmd_SESSION_CLEAR(Command())
        self.e.cmd_ROUGH_HOME(Command(ORIGINAL=1));self.assertEqual(self.e.optical_count,2)
    def test_rough_failure_restores_flags_and_does_not_publish_cold_datum(self):
        old=self.e._mechanics();self.e.failure='_KAMP_ROUGH_BASE ORIGINAL=1'
        with self.assertRaises(ValueError):self.e.cmd_ROUGH_HOME(Command(ORIGINAL=1))
        self.assertFalse(self.e.tool.G29_flag);self.assertEqual(self.e.za.zdown_switch_enable,0)
        self.assertEqual(self.e._mechanics(),old)
    def test_silent_optical_retry_exhaustion_rejected(self):
        self.e.bad_optical=True
        with self.assertRaises(ValueError):self.e.cmd_ROUGH_HOME(Command())
        self.assertFalse(self.e.preparation_ready)
    def test_unfinished_tilt_does_not_publish_mechanics(self):
        old=self.e._mechanics();self.e.cmd_ROUGH_HOME(Command());self.e.armed=True;self.e.skip_tilt=True
        with self.assertRaises(ValueError):self.e.cmd_HOT_PREPARE(Command())
        self.assertEqual(self.e._mechanics(),old)
    def test_counter_disagreement_rejected(self):
        self.e.cmd_ROUGH_HOME(Command());self.e.armed=True
        original=self.e._z_positions
        def corrupt():
            values=original();p,s=values['stepper_z1'];values['stepper_z1']=(p+100,s);return values
        self.e._z_positions=corrupt
        with self.assertRaises(ValueError):self.e.cmd_HOT_PREPARE(Command())
    def test_motor_off_invalidates_preparation(self):
        self.e.cmd_ROUGH_HOME(Command());self.e._motor_off(0);self.e.armed=True
        with self.assertRaises(ValueError):self.e.cmd_HOT_PREPARE(Command())
    def test_manual_z_motion_invalidates_preparation(self):
        self.e.cmd_ROUGH_HOME(Command());self.e.cmd_NOTE_Z_CHANGE(Command(STEPPER='stepper_z',DISTANCE=1))
        self.assertFalse(self.e.preparation_ready)
    def test_restore_real_stock_method_without_reprobing(self):
        e=self.prepared();self.assertIsNone(e.restore())
        self.assertTrue(e.active);self.assertFalse(e.recovering)
        self.assertEqual(e.optical_count,1);self.assertIn('model move',e.events)
        self.assertNotIn('G28 Z',e.events)
        self.assertLess(e.events.index('M190 S100'),e.events.index('G28 X Y'))
    def test_corrupt_snapshot_rejected_before_stock_start(self):
        e=self.prepared();Path(e.checkpoint_path).write_text('{}')
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
        self.assertFalse(any(x.startswith('_KAMP_SDCARD_BASE') for x in e.events))
    def test_modified_file_rejected(self):
        e=self.prepared();e.file.write_text('modified')
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_changed_configuration_rejected(self):
        e=self.prepared();e.fingerprint='different'
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_changed_mechanical_calibration_rejected(self):
        e=self.prepared();r.atomic_json(e.za.real_zmax_path,{'zmax':352.5})
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_stale_same_filename_job_generation_rejected(self):
        e=self.prepared();record=r.read_json(e.sd.print_file_name_path);record['kamp_job_id']='old-job'
        r.atomic_json(e.sd.print_file_name_path,record)
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_periodic_stock_record_keeps_generation(self):
        self.e.start_and_checkpoint();old=r.read_json(self.e.sd.print_file_name_path)['kamp_job_id']
        self.e.gm.recordPrintFileName(self.e.sd.print_file_name_path,str(self.e.file),filament_used=42)
        self.assertEqual(r.read_json(self.e.sd.print_file_name_path)['kamp_job_id'],old)
    def test_empty_recovery_parameter_refused_without_state_mutation(self):
        e=self.prepared()
        with self.assertRaises(r.RequestRefused):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=''))
        self.assertFalse(e.recovering);self.assertFalse(e.events)
    def test_nonempty_false_strings_keep_stock_recovery_semantics(self):
        e=self.prepared()
        for value in ('0','false','1'):
            e.sd.work_timer=None;e.recovering=False
            e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=value));self.assertTrue(e.recovering)
    def test_eeprom_before_start_or_outside_file_rejected(self):
        for position in (0,100000,-1,100.5):
            with self.subTest(position=position):
                e=self.prepared() if not Path(self.e.checkpoint_path).exists() else Engine(self.root)
                e.checkpoint_position['file_position']=position
                with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_second_stage_eeprom_mutation_stops_without_move(self):
        e=self.prepared();e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
        e.checkpoint_position['file_position']=0
        self.assertFalse(e._restore_stock({'file_position':0,'base_position_e':1},e.sd.print_file_name_path,{'Z':10}))
        self.assertTrue(e.sd.must_pause_work);self.assertEqual(e.optical_count,0)
        self.assertNotIn('model move',e.events)
    def test_stock_restore_exception_stops_before_model_move(self):
        e=self.prepared();e.failure='ZDOWN';e.restore()
        self.assertTrue(e.faulted);self.assertTrue(e.sd.must_pause_work)
        self.assertIsNone(e.sd.restore_print_timer);self.assertNotIn('model move',e.events)
        self.assertLess(e.events.index('SD stopped'),e.events.index('heaters off'))
    def test_silent_stock_early_return_is_failure(self):
        e=self.prepared();p=self.root/'box.json';r.atomic_json(str(p),{'enable':0})
        e.objects['box']=types.SimpleNamespace(box_state=types.SimpleNamespace(tn_save_data_path=str(p)),flush_material=lambda:None)
        e.gm.power_loss_sensor_detected=lambda:False;e.restore()
        self.assertTrue(e.faulted);self.assertNotIn('model move',e.events)
    def test_profile_load_before_optical_keeps_mesh_clear(self):
        e=self.prepared();e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
        e.cmd_RESTORE_SESSION(Command());self.assertIsNone(e.bedmesh.get_mesh())
    def test_cancel_invalidates_persisted_map(self):
        self.e.start_and_checkpoint();self.e.cmd_SESSION_CLEAR(Command())
        self.assertFalse(Path(self.e.checkpoint_path).exists());self.assertFalse(self.e.active)
    def test_new_preparation_invalidates_old_snapshot(self):
        self.e.start_and_checkpoint();self.e.active=False;self.e.sd.work_timer=None
        self.e.cmd_ROUGH_HOME(Command());self.assertFalse(Path(self.e.checkpoint_path).exists())
    def test_journal_repairs_powerloss_between_calibration_writes(self):
        value={'datum':{'zmax':352.2},'adjustment':{'stepper_z_adjustment':.2,'stepper_z1_adjustment':-.3}}
        original=r.atomic_json
        def fail(path,data):
            if path==self.e.za.real_zmax_path:raise OSError('power loss')
            return original(path,data)
        with patch.object(r,'atomic_json',side_effect=fail):
            with self.assertRaises(OSError):self.e._publish_mechanics(value)
        self.assertTrue(Path(self.e.transaction_path).exists())
        reboot=Engine(self.root);reboot._repair_mechanics()
        self.assertEqual(reboot._mechanics(),value);self.assertFalse(Path(reboot.transaction_path).exists())
    def test_atomic_snapshot_replace_failure_preserves_previous(self):
        p=str(self.root/'state/test.json');r.atomic_json(p,{'generation':1})
        with patch.object(r.os,'replace',side_effect=OSError('power loss')):
            with self.assertRaises(OSError):r.atomic_json(p,{'generation':2})
        self.assertEqual(r.read_json(p),{'generation':1})
    def test_corrupt_journal_fails_instead_of_loading_mixed_calibration(self):
        Path(self.e.state_dir).mkdir(exist_ok=True);Path(self.e.transaction_path).write_text('{}')
        with self.assertRaises(ValueError):self.e._repair_mechanics()
    def test_busy_new_request_leaves_active_print_intact(self):
        self.e.start_and_checkpoint();self.e.events.clear()
        with self.assertRaises(r.RequestRefused):self.e.cmd_START_FILE(Command(FILENAME='test.gcode'))
        self.assertTrue(self.e.active);self.assertFalse(self.e.events)
    def test_checkpoint_disabled_eeprom_refused(self):
        e=self.prepared();e.eeprom_enabled=False
        with self.assertRaises(ValueError):e.cmd_START_FILE(Command(FILENAME='test.gcode',ISCONTINUEPRINT=1))
    def test_stock_method_all_exception_handlers_reraise(self):
        node=method(GM,'GCodeMove','cmd_CX_RESTORE_GCODE_STATE')
        for h in [n for n in ast.walk(node) if isinstance(n,ast.ExceptHandler)]:self.assertIsInstance(h.body[-1],ast.Raise)
    def test_cancel_during_bed_heat_prevents_all_restore_motion(self):
        e=self.prepared();execute=e.gcode.run_script
        def cancel(script):
            execute(script)
            if script.startswith('M190'):e.cmd_SESSION_CLEAR(Command())
        e.gcode.run_script=cancel;e.restore()
        self.assertTrue(e.faulted);self.assertNotIn('G28 X Y',e.events);self.assertNotIn('model move',e.events)
    def test_cancel_after_stock_mutex_acquisition_prevents_motion(self):
        e=self.prepared()
        class CancelMutex:
            def __enter__(self):e.cmd_SESSION_CLEAR(Command())
            def __exit__(self,*args):pass
        e.gcode.get_mutex=lambda:CancelMutex();e.restore()
        self.assertTrue(e.faulted);self.assertNotIn('G28 X Y',e.events)
    def test_active_mesh_clear_and_offset_are_nonfatal_refusals(self):
        self.e.start_and_checkpoint();mesh=self.e.session_mesh
        for cmd in ('KAMP_CLEAR_MESH','KAMP_OFFSET_MESH'):
            with self.assertRaises(ValueError):self.e.commands[cmd](Command())
            self.assertFalse(self.e.faulted);self.assertIs(self.e.bedmesh.get_mesh(),mesh)
    def test_external_tilt_and_adjust_invalidate_prepared_baseline(self):
        for cmd in ('KAMP_TILT','KAMP_ADJUST'):
            self.e.cmd_ROUGH_HOME(Command())
            self.e.commands[cmd](Command())
            self.assertFalse(self.e.preparation_ready)
    def test_new_job_invalidates_enabled_old_eeprom_even_without_metadata(self):
        self.e.start_and_checkpoint();self.e.sd.work_timer=None;self.e.active=False
        Path(self.e.sd.print_file_name_path).unlink()
        self.assertFalse(self.e.eeprom.checkEepromFirstEnable())
        self.e.cmd_START_FILE(Command(FILENAME='test.gcode'))
        self.assertTrue(self.e.eeprom.checkEepromFirstEnable())
    def test_changed_open_file_descriptor_rejected(self):
        self.e.start_and_checkpoint();old=self.root/'old.gcode';old.write_text('different descriptor')
        self.e.sd.current_file.close()
        stream=open(old,encoding='utf-8')
        class ReplacedDescriptor:
            name=str(self.e.file)
            def fileno(self):return stream.fileno()
            def close(self):stream.close()
        self.e.sd.current_file=ReplacedDescriptor()
        with self.assertRaises(ValueError):self.e._save_checkpoint()
    def test_powerloss_at_each_eeprom_operation_never_accepts_mixed_record(self):
        class PowerLoss(BaseException):pass
        for cut in range(1,13):
            with self.subTest(cut=cut):
                e=Engine(self.root/('cut'+str(cut)));e.start_and_checkpoint()
                count=[0]
                def wrap(function):
                    def call(*args,**kw):
                        result=function(*args,**kw);count[0]+=1
                        if count[0]==cut:raise PowerLoss()
                        return result
                    return call
                names=('setEepromDisable','checkEepromFirstEnable','eepromReadHeader','eepromReadBody','write_reg')
                saved={name:getattr(e.eeprom,name) for name in names}
                atom=r.atomic_json
                try:
                    for name in names:setattr(e.eeprom,name,wrap(saved[name]))
                    with patch.object(r,'atomic_json',wrap(atom)):
                        try:e.commit_eeprom(e.eeprom,1500,15.)
                        except PowerLoss:pass
                finally:
                    for name in names:setattr(e.eeprom,name,saved[name])
                if e.eeprom.checkEepromFirstEnable():
                    with self.assertRaises(ValueError):e._verify_checkpoint(str(e.file))
                else:
                    value=e._verify_checkpoint(str(e.file));self.assertEqual(value['job_id'],e.job_id)
                    stored=r.checked_envelope(e.eeprom_path)
                    self.assertEqual(stored['info'],e.eeprom.eepromReadBody(e.eeprom.eepromReadHeader()))
    def runtime_environment(self,e,bad_target=False):
        def cls(name,path,method_names):
            return type(name,(),{n:compile_method(method(path,name,n)) for n in method_names})
        classes={
            'extras.z_align':('Zalign',SRC/'extras/z_align.py',('cmd_ZDOWN',)),
            'extras.homing':('PrinterHoming',SRC/'extras/homing.py',('write_real_zmax','cmd_G28')),
            'extras.homing_override':('HomingOverride',ROOT/'tests/fixtures/klipper/extras/homing_override.py',('cmd_G28',)),
            'extras.custom_macro':('CUSTOM_MACRO',SRC/'extras/custom_macro.py',('cmd_CX_ROUGH_G28',)),
            'stepper':('MCU_stepper',SRC/'stepper.py',('_query_mcu_position','get_mcu_position','set_position','get_step_dist')),
            'extras.gcode_move':('GCodeMove',GM,('cmd_CX_RESTORE_GCODE_STATE',)),
        }
        modules={}
        for module,(name,path,names) in classes.items():modules[module]=types.SimpleNamespace(**{name:cls(name,path,names)})
        ZT=cls('ZTilt',SRC/'extras/z_tilt.py',('cmd_Z_TILT_ADJUST','probe_finalize'))
        RH=cls('RetryHelper',SRC/'extras/z_tilt.py',('write_z_adjustment',))
        ZH=cls('ZAdjustHelper',SRC/'extras/z_tilt.py',('adjust_steppers',))
        # Probe start is a bound identity check; its behavior is covered by the
        # existing adapter's real upstream tests, not executed by this gate test.
        PH=type('ProbePointsHelper',(),{'start_probe':lambda self,cmd:None})
        modules['extras.z_tilt']=types.SimpleNamespace(ZTilt=ZT,RetryHelper=RH,ZAdjustHelper=ZH)
        modules['extras.probe']=types.SimpleNamespace(ProbePointsHelper=PH)
        e.za.cmd_ZDOWN=types.MethodType(modules['extras.z_align'].Zalign.cmd_ZDOWN,e.za)
        e.hm.write_real_zmax=types.MethodType(modules['extras.homing'].PrinterHoming.write_real_zmax,e.hm)
        e.gm.cmd_CX_RESTORE_GCODE_STATE=types.MethodType(modules['extras.gcode_move'].GCodeMove.cmd_CX_RESTORE_GCODE_STATE,e.gm)
        e.zt.cmd_Z_TILT_ADJUST=types.MethodType(ZT.cmd_Z_TILT_ADJUST,e.zt)
        e.zt.retry_helper=types.SimpleNamespace();e.zt.retry_helper.write_z_adjustment=types.MethodType(RH.write_z_adjustment,e.zt.retry_helper)
        e.zt.z_helper=types.SimpleNamespace();e.zt.z_helper.adjust_steppers=types.MethodType(ZH.adjust_steppers,e.zt.z_helper)
        e.zt.probe_helper=types.SimpleNamespace(finalize_callback=types.MethodType(ZT.probe_finalize,e.zt),start_probe=types.MethodType(PH.start_probe,object()))
        e.home_override=types.SimpleNamespace(in_script=False)
        e.home_override.cmd_G28=types.MethodType(modules['extras.homing_override'].HomingOverride.cmd_G28,e.home_override)
        e.home_override.prev_G28=types.MethodType(modules['extras.homing'].PrinterHoming.cmd_G28,e.hm)
        e.objects['homing_override']=e.home_override
        targets={'G28':e.home_override.cmd_G28,'_KAMP_TILT_BASE':e.zt.cmd_Z_TILT_ADJUST,
            '_KAMP_ROUGH_BASE':types.MethodType(modules['extras.custom_macro'].CUSTOM_MACRO.cmd_CX_ROUGH_G28,object())}
        register_fn=compile_method(method(SRC/'gcode.py','GCodeDispatch','register_command'))
        modules['gcode']=types.SimpleNamespace(GCodeDispatch=type('GCodeDispatch',(),{'register_command':register_fn}))
        e.gcode.ready_gcode_handlers={};e.gcode.gcode_help={}
        e.gcode.is_traditional_gcode=lambda name:name=='G28'
        e.gcode._get_extended_params=lambda cmd:cmd
        for name,target in targets.items():register_fn(e.gcode,name,target)
        if bad_target:e.gcode.ready_gcode_handlers['_KAMP_TILT_BASE']=lambda cmd:None
        SC=modules['stepper'].MCU_stepper
        for stepper in e.steppers:
            for name in ('_query_mcu_position','get_mcu_position','set_position','get_step_dist'):
                setattr(stepper,name,types.MethodType(getattr(SC,name),stepper))
        def local_open(path,mode='r',**kw):
            if path=='/mnt/UDISK/printer_data/config/sensorless.cfg':
                return open(ROOT/'tests/fixtures/config/sensorless.cfg',mode,**kw)
            rel=path.removeprefix('/usr/share/klipper/klippy/')
            target=PAYLOAD/'usr/share/klipper/klippy'/rel
            if not target.exists():target=SRC/rel
            if not target.exists():target=SRC/rel
            if not target.exists():
                if rel=='extras/bed_mesh.py':target=SRC/'extras/bed_mesh.py'
                else:target=SRC/rel
            return open(target,mode,**kw)
        def register(name,callback):
            previous=e.commands.get(name)
            if callback is None:return e.commands.pop(name,None)
            e.commands[name]=callback
            return previous
        e.commands['BED_MESH_SET_DISABLE']=lambda cmd:None
        e.commands['BED_MESH_SET_ENABLE']=lambda cmd:None
        e.gcode.register_command=register
        return modules,local_open
    def test_exact_runtime_gate_accepts_reviewed_callbacks(self):
        modules,opener=self.runtime_environment(self.e)
        with patch.dict(sys.modules,modules),patch.object(r,'open',opener,create=True):self.e._init_recovery()
    def test_runtime_gate_rejects_monkeypatched_dispatch(self):
        modules,opener=self.runtime_environment(self.e,bad_target=True)
        with patch.dict(sys.modules,modules),patch.object(r,'open',opener,create=True):
            with self.assertRaises(ValueError):self.e._init_recovery()
    def test_runtime_gate_rejects_changed_source_hash(self):
        modules,opener=self.runtime_environment(self.e)
        with patch.dict(sys.modules,modules),patch.object(r,'open',lambda *args,**kw:io.BytesIO(b'changed'),create=True):
            with self.assertRaises(ValueError):self.e._init_recovery()
    def test_runtime_gate_rejects_wrong_g28_bound_instance(self):
        for target in ('registry','override','previous'):
            modules,opener=self.runtime_environment(self.e)
            if target=='registry':
                self.e.gcode.ready_gcode_handlers['G28']=types.MethodType(modules['extras.homing_override'].HomingOverride.cmd_G28,object())
            elif target=='override':
                self.e.home_override.cmd_G28=types.MethodType(modules['extras.homing_override'].HomingOverride.cmd_G28,object())
            else:
                self.e.home_override.prev_G28=types.MethodType(modules['extras.homing'].PrinterHoming.cmd_G28,object())
            with self.subTest(target=target),patch.dict(sys.modules,modules),patch.object(r,'open',opener,create=True):
                with self.assertRaisesRegex(ValueError,'G28 override'):self.e._init_recovery()
    def test_exact_runtime_stepper_preserves_counter_on_set_position(self):
        node=method(SRC/'stepper.py','MCU_stepper','set_position')
        attrs=[n.attr for n in ast.walk(node) if isinstance(n,ast.Attribute)]
        self.assertIn('get_mcu_position',attrs);self.assertIn('_set_mcu_position',attrs)
    def test_macro_templates_and_both_start_paths(self):
        env=Environment(block_start_string='{%',block_end_string='%}',variable_start_string='{',variable_end_string='}')
        configs={}
        for path in PAYLOAD.rglob('*.cfg'):
            c=configparser.RawConfigParser(inline_comment_prefixes=('#',';'));c.read_string(path.read_text(encoding='utf-8'));configs[path.name]=c
            for section in c.sections():
                if c.has_option(section,'gcode'):env.parse(c.get(section,'gcode'))
        source=configs['gcode_macro.cfg'].get('gcode_macro START_PRINT','gcode')
        for prepared in (0,1):
            p={'custom_macro':{'g28_ext_temp':140,'default_bed_temp':60,'default_extruder_temp':220},'gcode_macro START_PRINT':{'prepare':prepared}}
            rendered=env.from_string(source).render(printer=p,params={'BED_TEMP':100,'EXTRUDER_TEMP':260},action_respond_info=lambda s:'')
            self.assertNotIn('ZDOWN_SWITCH ENABLE=1',rendered)
            self.assertEqual(rendered.count('KAMP_HOT_PREPARE'),1)
            self.assertEqual(rendered.count('KAMP_ROUGH_HOME'),1 if not prepared else 0)
            self.assertEqual(rendered.count('NOZZLE_CLEAR'),1 if not prepared else 0)
            self.assertIn('KAMP_ADAPTIVE_MESH',rendered)
        adaptive=configs['Adaptive_Meshing.cfg'].get('gcode_macro KAMP_ADAPTIVE_MESH','gcode')
        self.assertIn('min_points = 3',adaptive)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Cases))
    (OUT/'tests.json').write_text(json.dumps({'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'success':result.wasSuccessful(),'scope':'offline; actual saved homing, ZAdjustHelper and instrumented stock GCodeMove; mocked hardware/EEPROM'},indent=2),encoding='utf-8')
    sys.exit(not result.wasSuccessful())
