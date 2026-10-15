"""Offline integration checks using the printer's saved bed_mesh.py.

Only the physical probe helper and the C-backed mesh representation are mocked.
The saved upstream command, update_config, finalize and ProfileManager save code
are executed locally. No network or printer commands are issued.
"""
import sys,pathlib,types,importlib.util,configparser,unittest,json,copy,logging,ast
ROOT=pathlib.Path(__file__).resolve().parent.parent
from jinja2 import Environment
logging.disable(logging.CRITICAL)
OUT=ROOT
PROPOSED=OUT/'payload'
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);return module
sys.modules['extras']=types.ModuleType('extras');sys.modules['extras'].__path__=[]
sys.modules['extras.probe']=types.ModuleType('extras.probe')
sys.modules['mymodule']=types.ModuleType('mymodule');sys.modules['mymodule'].__path__=[]
sys.modules['mymodule.mymovie']=types.ModuleType('mymodule.mymovie')
sys.modules['numpy']=types.ModuleType('numpy')
upstream=load('extras.bed_mesh',ROOT/'tests/fixtures/klipper/extras/bed_mesh.py')
adapter=load('kamp_adapter_v2',PROPOSED/'usr/share/klipper/klippy/extras/restore_bed_mesh.py')

class GcodeError(Exception):pass
class Command:
    error=GcodeError
    def __init__(self,params=None):self.params={k.upper():str(v) for k,v in (params or {}).items()}
    def get(self,name,default=None):return self.params.get(name.upper(),default)
    def get_int(self,name,default=None,**kwargs):return int(self.get(name,default))
    def get_float(self,name,default=None,**kwargs):return float(self.get(name,default))
    def get_command_parameters(self):return self.params
    def get_raw_command_parameters(self):return ' '.join(k+'='+v for k,v in self.params.items())
    def respond_info(self,msg):pass

class Mesh:
    def __init__(self,params,printer=None):self.params=params;self.matrix=[]
    def build_mesh(self,matrix):self.matrix=matrix
    def get_probed_matrix(self):return self.matrix
    def get_mesh_params(self):return self.params
    def update_mesh_probed_matrix(self,matrix):self.matrix=matrix
upstream.ZMesh=Mesh
class Gcode:
    error=GcodeError
    def __init__(self):self.commands={};self.scripts=[];self.failure=None;self.on_script=None;self.attempts=[]
    def register_command(self,name,callback,**kwargs):
        self.attempts.append(name)
        if name in self.commands:raise GcodeError('duplicate command '+name)
        if self.failure==name:raise GcodeError('injected registration failure')
        self.commands[name]=callback
    def run_script_from_command(self,command):
        self.scripts.append(command)
        if self.on_script:self.on_script(command)
    def respond_info(self,msg):pass
class Printer:
    def __init__(self):self.objects={};self.shutdowns=[];self.events={}
    def lookup_object(self,name,default=None):return self.objects.get(name,default)
    def register_event_handler(self,event,callback):self.events[event]=callback
    def invoke_shutdown(self,message):self.shutdowns.append(message)
class Status:
    def __init__(self,**values):self.values=values
    def get_status(self,time):return self.values
class ConfigFile:
    def __init__(self):self.pending={}
    def set(self,section,name,value):self.pending.setdefault(section,{})[name]=value
class Heaters:
    def __init__(self):self.targets=[100,260,50];self.off_calls=0
    def turn_off_all_heaters(self):self.targets=[0,0,0];self.off_calls+=1
class Bed:
    update_mesh=upstream.BedMesh.update_mesh
    def __init__(self,events):self.mesh=None;self.events=events
    def get_mesh(self):return self.mesh
    def set_mesh(self,mesh):self.mesh=self.z_mesh=mesh;self.events.append('mesh_clear' if mesh is None else 'mesh_set')
    def update_status(self):pass
class VirtualSD:
    def __init__(self,events):
        self.events=events;self.work_timer=object();self.cmd_from_sd=True;self.must_pause_work=False
        self.reactor=types.SimpleNamespace(monotonic=lambda:0,pause=self.tick)
    def tick(self,time):self.work_timer=None;self.events.append('sd_stopped')
    def is_active(self):return self.work_timer is not None
    def next_line(self):
        if self.must_pause_work:return False
        self.events.append('next_file_command');return True
# Use the actual saved SD pause implementation, with a mocked reactor tick.
sd_tree=ast.parse((ROOT/'tests/fixtures/klipper/extras/virtual_sdcard.py').read_text(encoding='utf-8'))
sd_class=next(n for n in sd_tree.body if isinstance(n,ast.ClassDef) and n.name=='VirtualSD')
pause_ast=next(n for n in sd_class.body if isinstance(n,ast.FunctionDef) and n.name=='do_pause')
pause_namespace={};exec(compile(ast.Module(body=[pause_ast],type_ignores=[]),'saved_sd_pause','exec'),pause_namespace)
VirtualSD.do_pause=pause_namespace['do_pause']
class WebRequest:
    error=GcodeError
    def __init__(self,matrix):self.matrix=matrix;self.response=None
    def get(self,name,default=None):return self.matrix if name=='probed_matrix' else default
    def send(self,value):self.response=value
class ProbeHelper:
    def __init__(self,bmc):self.bmc=bmc;self.finalize_callback=bmc.probe_finalize;self.calls=0;self.fail=False;self.on_probe=None
    def update_probe_points(self,points,count):self.points=points
    def start_probe(self,gcmd):
        self.calls+=1
        if self.fail:raise GcodeError('injected probe failure')
        if self.on_probe:self.on_probe()
        self.finalize_callback((0,0,0),[[x,y,0] for x,y in self.points])
sys.modules['extras.probe'].ProbePointsHelper=ProbeHelper
class Config:
    def __init__(self,printer):self.printer=printer
    def get_printer(self):return self.printer

def fixture(ready=True,fail_registration=None):
    events=[];printer=Printer();gcode=Gcode();gcode.failure=fail_registration;bed=Bed(events);heaters=Heaters();pending=ConfigFile();sd=VirtualSD(events)
    bed.gcode=gcode
    objects=[{'polygon':[[155,155],[195,155],[195,195],[155,195]]}]
    printer.objects={'gcode':gcode,'bed_mesh':bed,'heaters':heaters,'configfile':pending,
                     'print_stats':Status(state='printing'),'exclude_object':Status(objects=objects),
                     'virtual_sdcard':sd,'filament_switch_sensor filament_sensor':Status(enabled=True),
                     'webhooks':types.SimpleNamespace(_endpoints={'update_mesh':bed.update_mesh})}
    bmc=upstream.BedMeshCalibrate.__new__(upstream.BedMeshCalibrate)
    bmc.printer=printer;bmc.gcode=gcode;bmc.bedmesh=bed
    bmc.radius=bmc.origin=bmc.relative_reference_index=None
    bmc.faulty_regions=[];bmc.substituted_indices={}
    bmc.mesh_config={'x_count':9,'y_count':9,'mesh_x_pps':2,'mesh_y_pps':2,'algo':'bicubic','tension':0.2}
    bmc.orig_config=dict(bmc.mesh_config,radius=None,origin=None,rri=None,mesh_min=(5,5),mesh_max=(345,345))
    bmc.orig_points=[];bmc.probe_helper=ProbeHelper(bmc);bed.bmc=bmc
    pmgr=upstream.ProfileManager.__new__(upstream.ProfileManager)
    pmgr.printer=printer;pmgr.bedmesh=bed;pmgr.gcode=gcode;pmgr.name='bed_mesh'
    pmgr.profiles={'default':{'points':'original full-bed map'}};pmgr.current_profile='default'
    bed.pmgr=pmgr;bed.save_profile=pmgr.save_profile
    bed.load_profile=lambda name:events.append('profile_load_'+name)
    mod=adapter.BedMeshOverride(Config(printer))
    if ready:mod._handle_ready()
    return types.SimpleNamespace(printer=printer,gcode=gcode,bed=bed,mod=mod,heaters=heaters,pending=pending,pmgr=pmgr,bmc=bmc,sd=sd,events=events)
def run(f,name,**params):return f.gcode.commands[name](Command(params))
def arm(f):run(f,'KAMP_VALIDATE_START',MESH=1)
def probe(f,**overrides):
    params=dict(MODE='ADAPTIVE',PROFILE='_kamp_session',MESH_MIN='150,150',MESH_MAX='200,200',PROBE_COUNT='3,3',ALGORITHM='lagrange');params.update(overrides)
    return run(f,'_KAMP_PROBE',**params)

env=Environment(block_start_string='{%',block_end_string='%}',variable_start_string='{',variable_end_string='}')
configs={}
for path in PROPOSED.rglob('*.cfg'):
    cfg=configparser.RawConfigParser(strict=True,inline_comment_prefixes=('#',';'));cfg.read_string(path.read_text(encoding='utf-8'));configs[path.name]=cfg
def render(filename,section,printer,params=None):
    params={k.upper():str(v) for k,v in (params or {}).items()}
    source=configs[filename].get(section,'gcode')
    return [s.strip() for s in env.from_string(source).render(printer=printer,params=params,rawparams=' '.join(k+'='+v for k,v in params.items()),action_respond_info=lambda msg:'').splitlines() if s.strip() and not s.strip().startswith('#')]

def install_manual_executor(f,failure_stage=None):
    """Render the actual private macro, dispatch its commands into local stubs."""
    sensor=f.printer.objects['filament_switch_sensor filament_sensor']
    def execute(command):
        token=command.split()[0]
        if token=='_KAMP_FULL_SEQUENCE':
            p={'custom_macro':{'default_bed_temp':60,'g28_ext_temp':140,'default_extruder_temp':220},'filament_switch_sensor filament_sensor':sensor.values,'toolhead':{'homed_axes':'xyz'}}
            for line in render('Full_Bed_Calibrate.cfg','gcode_macro _KAMP_FULL_SEQUENCE',p):execute(line)
            return
        f.events.append('command '+token)
        if token==failure_stage:raise GcodeError('injected '+token+' failure')
        if token=='SET_FILAMENT_SENSOR':sensor.values['enabled']=command.endswith('=1')
        if token=='_KAMP_PROBE':
            params=dict(part.split('=',1) for part in command.split()[1:])
            run(f,token,**params)
    f.gcode.on_script=execute

class AdapterTests(unittest.TestCase):
    def test_ready_and_status(self):
        f=fixture();self.assertTrue(f.mod.get_status(0)['ready']);self.assertFalse(f.printer.shutdowns)
    def test_registration_failure_propagates(self):
        with self.assertRaisesRegex(GcodeError,'injected registration failure'):fixture(fail_registration='_KAMP_PROBE')
    def test_unknown_dynamic_method_shutdown(self):
        f=fixture(False);f.bmc.update_config=lambda cmd:None;f.mod._handle_ready()
        self.assertFalse(f.mod.ready);self.assertTrue(f.printer.shutdowns)
    def test_unknown_callback_shutdown(self):
        f=fixture(False);f.bmc.probe_helper.finalize_callback=lambda offsets,positions:None;f.mod._handle_ready()
        self.assertFalse(f.mod.ready);self.assertTrue(f.printer.shutdowns)
    def test_unknown_probe_start_shutdown(self):
        f=fixture(False);f.bmc.probe_helper.start_probe=lambda cmd:None;f.mod._handle_ready()
        self.assertFalse(f.mod.ready);self.assertTrue(f.printer.shutdowns)
    def test_missing_bmc_shutdown(self):
        f=fixture(False);f.bed.bmc=None;f.mod._handle_ready();self.assertTrue(f.printer.shutdowns)
    def test_not_ready_stops_heaters(self):
        f=fixture(False)
        with self.assertRaises(GcodeError):arm(f)
        self.assertEqual(f.heaters.targets,[0,0,0])
    def test_missing_metadata_turns_off_existing_heaters(self):
        f=fixture();f.printer.objects['exclude_object'].values['objects']=[]
        with self.assertRaisesRegex(GcodeError,'no object boundaries'):arm(f)
        self.assertEqual(f.heaters.targets,[0,0,0]);self.assertIn('PRINT_PREPARE_CLEAR',f.gcode.scripts)
    def test_invalid_polygon_rejected(self):
        f=fixture();f.printer.objects['exclude_object'].values['objects']=[{'polygon':[[10,10],[20,20]]}]
        with self.assertRaises(GcodeError):arm(f)
        self.assertFalse(f.mod.armed)
    def test_mesh_skip_rejected(self):
        f=fixture()
        with self.assertRaises(GcodeError):run(f,'KAMP_VALIDATE_START',MESH=0)
    def test_manual_start_rejected(self):
        f=fixture();f.printer.objects['print_stats'].values['state']='complete'
        with self.assertRaises(GcodeError):arm(f)
    def test_probe_needs_explicit_one_use_start_even_with_stale_objects(self):
        f=fixture()
        with self.assertRaises(GcodeError):probe(f)
        self.assertEqual(f.bmc.probe_helper.calls,0)
    def test_adaptive_uses_real_upstream_and_never_saves_default(self):
        f=fixture();before=copy.deepcopy(f.pmgr.profiles);arm(f);probe(f)
        self.assertTrue(f.mod.active);self.assertFalse(f.mod.armed)
        self.assertEqual(f.bmc.probe_helper.calls,1);self.assertEqual(f.pmgr.profiles,before)
        self.assertEqual(f.pending.pending,{});self.assertNotIn('_kamp_session',f.pmgr.profiles)
        self.assertEqual(f.bmc.mesh_config['x_count'],3)
    def test_repeat_probe_blocked(self):
        f=fixture();arm(f);probe(f)
        with self.assertRaises(GcodeError):probe(f)
        self.assertEqual(f.bmc.probe_helper.calls,1)
    def test_probe_failure_restores_save_hook_and_stops_heat(self):
        f=fixture();original=f.bed.save_profile;arm(f);f.bmc.probe_helper.fail=True
        with self.assertRaisesRegex(GcodeError,'probe failure'):probe(f)
        self.assertEqual(f.bed.save_profile,original);self.assertEqual(f.pending.pending,{})
        self.assertFalse(f.mod.active);self.assertEqual(f.heaters.targets,[0,0,0])
    def test_wrong_profile_cannot_persist_adaptive(self):
        f=fixture();arm(f)
        with self.assertRaises(GcodeError):probe(f,PROFILE='default')
        self.assertEqual(f.pending.pending,{})
    def test_bad_bounds_stops_without_probing(self):
        for bounds in ['nan,5','0,5','345,345']:
            f=fixture();arm(f)
            with self.assertRaises(GcodeError):probe(f,MESH_MIN=bounds)
            self.assertEqual(f.bmc.probe_helper.calls,0)
    def test_restore_session_after_mesh_clear(self):
        f=fixture();arm(f);probe(f);mesh=f.mod.session_mesh;f.bed.set_mesh(None)
        run(f,'KAMP_RESTORE_SESSION');self.assertIs(f.bed.get_mesh(),mesh)
    def test_session_end_discards_ephemeral_map_only(self):
        f=fixture();arm(f);probe(f);run(f,'KAMP_SESSION_CLEAR')
        self.assertFalse(f.mod.active);self.assertIsNone(f.bed.get_mesh());self.assertIn('default',f.pmgr.profiles)
    def test_manual_full_probe_saves_real_default(self):
        f=fixture();f.printer.objects['print_stats'].values['state']='complete';f.sd.work_timer=None;run(f,'KAMP_VALIDATE_FULL')
        f.mod.manual_busy=f.mod.full_armed=True
        probe(f,MODE='FULL',PROFILE='default',MESH_MIN='5,5',MESH_MAX='345,345',PROBE_COUNT='9,9',ALGORITHM='bicubic')
        self.assertIn('bed_mesh default',f.pending.pending);self.assertFalse(f.mod.active)
    def test_manual_full_rejected_before_macro_moves_when_printing(self):
        f=fixture()
        with self.assertRaises(GcodeError):run(f,'KAMP_VALIDATE_FULL')
        self.assertEqual(f.bmc.probe_helper.calls,0)
    def test_external_profile_refusal_and_recovery_preserve_running_job(self):
        for command in ('_KAMP_REFUSE_PROFILE','_KAMP_REJECT_RECOVERY'):
            f=fixture();arm(f);probe(f);mesh=f.bed.get_mesh();targets=list(f.heaters.targets)
            with self.assertRaises(GcodeError):run(f,command)
            self.assertEqual(f.heaters.targets,targets);self.assertIs(f.bed.get_mesh(),mesh);self.assertTrue(f.mod.active)
            self.assertTrue(f.sd.next_line());self.assertFalse(f.mod.faulted)
    def test_abort_idempotent(self):
        f=fixture();run(f,'KAMP_ABORT');run(f,'KAMP_ABORT');self.assertFalse(f.mod.active)
    def test_fatal_abort_stops_file_before_discarding_compensation(self):
        f=fixture();arm(f);probe(f);f.sd.cmd_from_sd=False;f.events.clear();run(f,'KAMP_ABORT')
        self.assertLess(f.events.index('sd_stopped'),f.events.index('mesh_clear'));self.assertFalse(f.sd.next_line())
        self.assertTrue(f.mod.faulted);self.assertEqual(f.heaters.targets,[0,0,0])
    def test_fatal_from_inside_file_requests_pause_and_prevents_next_line(self):
        f=fixture();f.printer.objects['exclude_object'].values['objects']=[]
        with self.assertRaises(GcodeError):arm(f)
        self.assertTrue(f.sd.must_pause_work);self.assertFalse(f.sd.next_line())
    def test_cannot_resume_after_fatal_cleanup(self):
        f=fixture();run(f,'KAMP_ABORT')
        with self.assertRaises(GcodeError):run(f,'KAMP_CHECK_RESUME')
        self.assertFalse(f.sd.next_line())
    def test_new_file_clears_fault_only_after_previous_job_stopped(self):
        f=fixture();run(f,'KAMP_ABORT');f.sd.work_timer=None;f.printer.objects['print_stats'].values['state']='error'
        run(f,'KAMP_PREPARE_FILE');self.assertFalse(f.mod.faulted)
    def test_normal_pause_resume_keeps_map(self):
        f=fixture();arm(f);probe(f);mesh=f.bed.get_mesh();run(f,'KAMP_CHECK_RESUME');self.assertIs(f.bed.get_mesh(),mesh)
    def test_rejected_manual_full_leaves_running_job_unchanged(self):
        f=fixture();arm(f);probe(f);mesh=f.bed.get_mesh();targets=list(f.heaters.targets)
        with self.assertRaises(GcodeError):run(f,'KAMP_FULL_BED_CALIBRATE')
        self.assertIs(f.bed.get_mesh(),mesh);self.assertEqual(f.heaters.targets,targets);self.assertTrue(f.sd.next_line())
    def test_all_manual_sequence_failures_restore_sensor_and_turn_heat_off(self):
        for stage in ('G28','NOZZLE_CLEAR','Z_TILT_ADJUST','_KAMP_PROBE','CXSAVE_CONFIG'):
            for initially_enabled in (False,True):
                f=fixture();f.sd.work_timer=None;f.printer.objects['print_stats'].values['state']='complete'
                sensor=f.printer.objects['filament_switch_sensor filament_sensor'];sensor.values['enabled']=initially_enabled
                install_manual_executor(f,stage)
                with self.assertRaisesRegex(GcodeError,stage):run(f,'KAMP_FULL_BED_CALIBRATE')
                self.assertEqual(sensor.values['enabled'],initially_enabled);self.assertEqual(f.heaters.targets,[0,0,0]);self.assertFalse(f.mod.manual_busy)
    def test_manual_success_restores_original_disabled_sensor(self):
        f=fixture();f.sd.work_timer=None;f.printer.objects['print_stats'].values['state']='complete'
        sensor=f.printer.objects['filament_switch_sensor filament_sensor'];sensor.values['enabled']=False
        install_manual_executor(f);run(f,'KAMP_FULL_BED_CALIBRATE')
        self.assertIn('SET_FILAMENT_SENSOR SENSOR=filament_sensor ENABLE=0',f.gcode.scripts)
        self.assertEqual(f.bmc.probe_helper.calls,1);self.assertIn('bed_mesh default',f.pending.pending)
    def test_webhook_rejects_before_touching_active_mesh(self):
        f=fixture();arm(f);probe(f);mesh=f.bed.get_mesh();matrix=copy.deepcopy(mesh.matrix)
        request=WebRequest([[9]*3]*3)
        with self.assertRaises(GcodeError):f.printer.objects['webhooks']._endpoints['update_mesh'](request)
        self.assertEqual(mesh.matrix,matrix);self.assertEqual(f.pending.pending,{});self.assertTrue(f.mod.active);self.assertTrue(f.sd.next_line())
    def test_webhook_blocked_while_adaptive_probe_has_consumed_its_token(self):
        f=fixture();arm(f)
        def during_probe():
            self.assertFalse(f.mod.armed);self.assertFalse(f.mod.active);self.assertTrue(f.mod.probing)
            with self.assertRaises(GcodeError):f.printer.objects['webhooks']._endpoints['update_mesh'](WebRequest([[9]]))
        f.bmc.probe_helper.on_probe=during_probe;probe(f);self.assertTrue(f.mod.active)
    def test_actual_saved_webhook_runs_normally_outside_session(self):
        f=fixture();f.sd.work_timer=None;f.printer.objects['print_stats'].values['state']='complete';f.mod.manual_busy=f.mod.full_armed=True
        probe(f,MODE='FULL',PROFILE='default',MESH_MIN='5,5',MESH_MAX='345,345',PROBE_COUNT='9,9',ALGORITHM='bicubic');f.mod.manual_busy=False
        matrix=[[1]*9 for _ in range(9)];request=WebRequest(matrix)
        f.printer.objects['webhooks']._endpoints['update_mesh'](request)
        self.assertEqual(f.pmgr.profiles['default']['points'],matrix);self.assertIn('CXSAVE_CONFIG',f.gcode.scripts)
    def test_unknown_webhook_registration_blocks_ready(self):
        f=fixture(False);f.printer.objects['webhooks']._endpoints['update_mesh']=lambda request:None;f.mod._handle_ready()
        self.assertFalse(f.mod.ready);self.assertTrue(f.printer.shutdowns)

class MacroTests(unittest.TestCase):
    def test_all_templates_parse(self):
        for cfg in configs.values():
            for section in cfg.sections():
                if cfg.has_option(section,'gcode'):env.parse(cfg.get(section,'gcode'))
    def test_rename_types_match_the_actual_saved_gcode_dispatcher(self):
        tree=ast.parse((ROOT/'tests/fixtures/klipper/gcode.py').read_text(encoding='utf-8'))
        method=next(n for cls in tree.body if isinstance(cls,ast.ClassDef) for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='is_traditional_gcode')
        namespace={};exec(compile(ast.Module(body=[method],type_ignores=[]),'saved_gcode_types','exec'),namespace)
        classify=lambda name:namespace['is_traditional_gcode'](None,name)
        for cfg in configs.values():
            for section in cfg.sections():
                if cfg.has_option(section,'rename_existing'):
                    alias=section.split()[-1];renamed=cfg.get(section,'rename_existing')
                    self.assertEqual(classify(alias),classify(renamed),(alias,renamed))
    def test_early_calibration_never_reaches_probe_with_stale_objects(self):
        for filename,name in [('Guarded_Commands.cfg','BED_MESH_CALIBRATE'),('gcode_macro.cfg','G29'),('gcode_macro.cfg','BED_MESH_CALIBRATE_START_PRINT')]:
            commands=render(filename,'gcode_macro '+name,{'exclude_object':{'objects':[{'polygon':[[155,155],[195,155],[195,195]]}]}},{'BED_TEMP':100})
            self.assertTrue(all(c.startswith('M118') for c in commands),commands)
    def test_profile_load_restores_session_after_homing(self):
        commands=render('Guarded_Commands.cfg','gcode_macro BED_MESH_PROFILE',{'restore_bed_mesh':{'active':True}},{'LOAD':'default'})
        self.assertEqual(commands,['KAMP_RESTORE_SESSION'])
    def test_profile_save_and_remove_blocked_during_session(self):
        for params in ({'SAVE':'my_mesh'},{'REMOVE':'default'},{'LOAD':'another'}):
            self.assertEqual(render('Guarded_Commands.cfg','gcode_macro BED_MESH_PROFILE',{'restore_bed_mesh':{'active':True}},params),['_KAMP_REFUSE_PROFILE'])
    def test_profile_outside_session_passes_to_stock(self):
        self.assertEqual(render('Guarded_Commands.cfg','gcode_macro BED_MESH_PROFILE',{'restore_bed_mesh':{'active':False}},{'LOAD':'default'}),['_KAMP_PROFILE_BASE LOAD=default'])
    def test_recovery_rejected_before_stock_start(self):
        for value in ('0','false','none','1','true',''):
            self.assertEqual(render('Guarded_Commands.cfg','gcode_macro SDCARD_PRINT_FILE',{}, {'ISCONTINUEPRINT':value}),['_KAMP_REJECT_RECOVERY'])
        self.assertEqual(render('Guarded_Commands.cfg','gcode_macro SDCARD_PRINT_FILE',{}, {'FILENAME':'test.gcode'}),['KAMP_PREPARE_FILE','_KAMP_SDCARD_BASE FILENAME=test.gcode'])
    def test_real_saved_recovery_assignment_treats_false_strings_as_truthy(self):
        handler=next(n for n in sd_class.body if isinstance(n,ast.FunctionDef) and n.name=='cmd_SDCARD_PRINT_FILE')
        assignment=next(n for n in handler.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='is_continue_print' for t in n.targets))
        code=compile(ast.Module(body=[assignment],type_ignores=[]),'saved_recovery_assignment','exec')
        for value in ('0','false','none','1','true'):
            obj=types.SimpleNamespace();exec(code,{'self':obj,'gcmd':Command({'ISCONTINUEPRINT':value})});self.assertTrue(obj.is_continue_print)
    def test_resume_commands_all_check_fault_before_base(self):
        commands=render('Guarded_Commands.cfg','gcode_macro M24',{})
        self.assertEqual(commands,['KAMP_CHECK_RESUME','M24.1'])
        self.assertTrue(configs['gcode_macro.cfg'].get('gcode_macro RESUME','gcode').lstrip().startswith('KAMP_CHECK_RESUME'))
    def test_start_both_prepare_branches_keep_complete_stock_tilt_flags(self):
        for prepared in (0,1):
            p={'custom_macro':{'g28_ext_temp':140,'default_bed_temp':60,'default_extruder_temp':220},'gcode_macro START_PRINT':{'prepare':prepared}}
            commands=render('gcode_macro.cfg','gcode_macro START_PRINT',p,{'BED_TEMP':100,'EXTRUDER_TEMP':260})
            self.assertEqual(commands[0],'KAMP_VALIDATE_START MESH=1')
            self.assertEqual(commands.count('NOZZLE_CLEAR'), 1 if prepared == 0 else 0)
            self.assertEqual(commands.count('G28 Z'), 3 if prepared == 0 else 2)
            expected=['SET_G29_FLAG VALUE=1','BED_MESH_CLEAR','M104 S140','M190 S100.0','M109 S140','ZDOWN_SWITCH ENABLE=1','G28 Z','ZDOWN_SWITCH ENABLE=0','BED_MESH_CLEAR','Z_TILT_ADJUST WAITTIME=5','NEXT_HOMEZ_NACCU','G28 Z','SET_G29_FLAG VALUE=0','KAMP_ADAPTIVE_MESH']
            filtered=[c for c in commands if not c.startswith('#')]
            start=filtered.index('SET_G29_FLAG VALUE=1')
            self.assertEqual(filtered[start:start+len(expected)],expected)
            self.assertFalse(any('LINE_PURGE' in c for c in commands))
    def test_full_sequence_is_private_and_wrapped_by_python_command(self):
        p={'custom_macro':{'default_bed_temp':60,'g28_ext_temp':140,'default_extruder_temp':220},'filament_switch_sensor filament_sensor':{'enabled':True},'toolhead':{'homed_axes':'xyz'}}
        self.assertTrue(configs['Full_Bed_Calibrate.cfg'].has_section('gcode_macro _KAMP_FULL_SEQUENCE'))
        self.assertFalse(configs['Full_Bed_Calibrate.cfg'].has_section('gcode_macro KAMP_FULL_BED_CALIBRATE'))
    def test_sd_error_hook_is_abort_command(self):
        self.assertEqual(configs['printer.cfg'].get('virtual_sdcard','on_error_gcode').strip(),'KAMP_ABORT')

# Appended to the existing offline regression suite by build_kamp_v23.py.
# Executes the device's actual cmd_G28; only motors/kinematics are substituted.
class HomingRoutingTests(unittest.TestCase):
    def run_sequence(self, already, consume):
        events=[]
        tree=ast.parse((ROOT/'tests/fixtures/klipper/extras/homing.py').read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PrinterHoming')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='cmd_G28')
        class HomingState:
            def __init__(self,printer):self.out_z_all=0
            def set_axes(self,axes):self.axes=axes
        namespace={'Homing':HomingState,'MOTOR_PROTECT_ERROR':-10001,'logging':logging}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_saved_cmd_G28','exec'),namespace)
        z=types.SimpleNamespace(is_already_zodwn=already,zdown_switch_enable=0,pin_len=2,force_stop_flag=False)
        def script(command):
            events.append(command)
        gcode=types.SimpleNamespace(run_script_from_command=script,respond_info=lambda msg:None)
        def home(state):
            state.out_z_all=0.25
            events.append('kin.home')
        kin=types.SimpleNamespace(home=home)
        toolhead=types.SimpleNamespace(G29_flag=True,get_max_accel=lambda:5000,get_kinematics=lambda:kin,get_status=lambda now:{'homed_axes':'xyz'})
        objects={'toolhead':toolhead,'gcode':gcode,'z_align':z,'gcode_move':types.SimpleNamespace(get_status=lambda now:{'position':[175,175,5,0]})}
        printer=types.SimpleNamespace(lookup_object=lambda name,default=None:objects.get(name,default),get_reactor=lambda:types.SimpleNamespace(monotonic=lambda:1),command_error=GcodeError,is_shutdown=lambda:False)
        def two_z():
            events.append('optical ZDOWN')
            return 0
        driver=types.SimpleNamespace(printer=printer,probe_type='',config=types.SimpleNamespace(has_section=lambda name:name=='z_align'),z_move=54,run_gcmd=lambda command,**kw:script(command),run_G28_two_Z=two_z,write_real_zmax=lambda value:events.append('write_real_zmax'))
        call=lambda:namespace['cmd_G28'](driver,Command({'Z':0}))
        z.zdown_switch_enable=1
        call()
        after_first_switch=z.zdown_switch_enable
        if consume:
            # Execute the actual device's ZDOWN_SWITCH setter too.
            ztree=ast.parse((ROOT/'tests/fixtures/klipper/extras/z_align.py').read_text(encoding='utf-8'))
            zcls=next(n for n in ztree.body if isinstance(n,ast.ClassDef) and n.name=='Zalign')
            setter=next(n for n in zcls.body if isinstance(n,ast.FunctionDef) and n.name=='cmd_ZDOWN_SWITCH')
            zns={};exec(compile(ast.Module(body=[setter],type_ignores=[]),'actual_ZDOWN_SWITCH','exec'),zns)
            zns['cmd_ZDOWN_SWITCH'](z,Command({'ENABLE':0}))
        events.append('Z_TILT_ADJUST')
        call()
        return events,after_first_switch,z
    def test_actual_homing_consumes_optical_request_for_both_initial_states(self):
        for already in (False,True):
            with self.subTest(already=already):
                events,_,z=self.run_sequence(already,True)
                tilt=events.index('Z_TILT_ADJUST')
                self.assertEqual(events[:tilt].count('optical ZDOWN'),1)
                self.assertEqual(events[tilt:].count('optical ZDOWN'),0)
                self.assertEqual(events[tilt:].count('kin.home'),1)
                self.assertEqual(z.zdown_switch_enable,0)
                self.assertTrue(z.is_already_zodwn)
    def test_actual_first_initialization_leaves_request_without_explicit_consumption(self):
        events,switch,_=self.run_sequence(False,False)
        self.assertEqual(switch,1)
        self.assertEqual(events[events.index('Z_TILT_ADJUST'):].count('optical ZDOWN'),1)


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report={'tests_run':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'success':result.wasSuccessful(),'scope':'offline; real saved bed_mesh command/update/finalize/save code, mocked probe and C-backed ZMesh; no hardware validation'}
    (OUT/'tests/results').mkdir(exist_ok=True)
    (OUT/'tests/results/latest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    sys.exit(not result.wasSuccessful())
