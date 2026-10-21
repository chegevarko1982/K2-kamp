"""Actual HomingOverride and firmware G28 under the observed LAN sequence."""
import configparser,importlib.util,json,pathlib,tempfile,types,unittest
ROOT=pathlib.Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('integration241',ROOT/'tests/test_recovery.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
OVERRIDE=b.compile_method(b.method(ROOT/'tests/fixtures/klipper/extras/homing_override.py','HomingOverride','cmd_G28'))
class Cases(unittest.TestCase):
 def setUp(self):
  (b.OUT/'tmp').mkdir(exist_ok=True);self.temp=tempfile.TemporaryDirectory(dir=b.OUT/'tmp');self.e=e=b.Engine(self.temp.name)
  self.now=10.;e.printer.get_reactor=lambda:types.SimpleNamespace(monotonic=lambda:self.now)
  self.raw=[];self.nested=[]
  self.original_execute=e.execute
  def base_home(cmd):
   self.nested.append(cmd.get_command_parameters())
   if 'Z' in cmd.get_command_parameters():self.original_execute('G28 Z')
  self.homed='';self.xyz=dict(x_ready=0,y_ready=0,z_ready=0,xy_moved=0,z_moved=0,safe_z=3)
  cfg=configparser.RawConfigParser(inline_comment_prefixes=('#',';'))
  cfg.read(ROOT/'tests/fixtures/config/sensorless.cfg',encoding='utf-8')
  env=b.Environment(block_start_string='{%',block_end_string='%}',variable_start_string='{',variable_end_string='}')
  def render_sensorless(section,params):
   printer={'toolhead':{'homed_axes':self.homed,'position':dict(zip('xyze',e.position)),'max_accel':5000},
    'gcode_macro xyz_ready':self.xyz,'print_stats':{'z_pos':0,'power_loss':0},
    'configfile':{'settings':{'bed_mesh':{'mesh_min':[5,5],'mesh_max':[345,345]},
     'stepper_x':{'position_max':352.5},'stepper_y':{'position_max':352}}}}
   text=env.from_string(cfg.get(section,'gcode')).render(printer=printer,params=params,action_respond_info=lambda text:'')
   for line in text.splitlines():
    line=line.strip()
    if not line or line.startswith('#'):continue
    parts=line.split();name=parts[0]
    if cfg.has_section('gcode_macro '+name):render_sensorless('gcode_macro '+name,{})
    elif name=='G28':
     axes={p[0]:p[1:] for p in parts[1:]}
     e._dispatch_home(b.Command(**axes))
     self.homed=''.join(c for c in 'xyz' if c in self.homed or c.upper() in axes)
    elif name=='SET_GCODE_VARIABLE':
     values=dict(part.split('=',1) for part in parts[1:]);self.xyz[values['VARIABLE']]=int(values['VALUE'])
    elif name=='G1' and any(p.startswith('Z') for p in parts[1:]):
     p=list(e.position);p[2]=float(next(p[1:] for p in parts[1:] if p.startswith('Z')));e.move(p,10)
    elif name=='BED_MESH_CLEAR':e.bedmesh.set_mesh(None)
    # Non-Z motion, pressure sensor, fan and driver commands are hardware stubs.
  def template(context):
   self.raw.append(context['params'])
   render_sensorless('homing_override',context['params'])
  ho=e.home_override=types.SimpleNamespace(config=types.SimpleNamespace(has_section=lambda name:False),
   printer=e.printer,gcode=e.gcode,in_script=False,axes='XYZ',start_pos=[None,None,None],prev_G28=base_home,
   template=types.SimpleNamespace(create_template_context=lambda:{},run_gcode_from_command=template))
  e._stock_g28=types.MethodType(OVERRIDE,ho)
  def execute(script):
   for line in script.splitlines():
    if line in ('G28','G28 Z'):e._dispatch_home(b.Command(**({'Z':''} if line=='G28 Z' else {})))
    else:self.original_execute(line)
  e.gcode.run_script_from_command=execute;e.gcode.run_script=execute
  e.objects['gcode_macro START_PRINT']=types.SimpleNamespace(get_status=lambda now:{'prepare':1})
  e.objects['exclude_object']=types.SimpleNamespace(get_status=lambda now:{'objects':[{'polygon':[[10,10],[60,10],[60,60]]}]})
 def tearDown(self):
  for e in b.Engine.instances:
   if e.sd.current_file:e.sd.current_file.close()
  b.Engine.instances.clear();self.temp.cleanup()
 def lan_home(self):
  self.e.cmd_LAN_PREPARE(b.Command(WAIT_TEMP=0));self.e._dispatch_home(b.Command(Z=''))
 def test_observed_daemon_sequence_then_hot_home_has_one_optical(self):
  e=self.e;self.lan_home()
  self.assertEqual(self.raw,[{'Z':''}]);self.assertEqual(len(self.nested),3)
  self.assertTrue(e.preparation_ready);self.assertEqual(e.optical_count,1)
  self.assertFalse(e.tool.G29_flag);self.assertEqual(e.za.zdown_switch_enable,0)
  e.cmd_LAN_PREPARE(b.Command(WAIT_TEMP=1));e.state='printing'
  e.cmd_VALIDATE_START(b.Command());e.cmd_HOT_PREPARE(b.Command())
  self.assertEqual(e.optical_count,1)
  self.assertAlmostEqual(b.r.read_json(e.za.real_zmax_path)['zmax'],e.hot_datum)
 def test_unmarked_manual_home_uses_stock_mode(self):
  self.e._dispatch_home(b.Command(Z=''));self.assertFalse(self.e.preparation_ready)
  self.assertFalse(self.e.tool.G29_flag);self.assertIsNone(self.e._lan_home_deadline)
 def test_upper_home_without_axes_consumes_one_token(self):
  self.e.cmd_LAN_PREPARE(b.Command());self.e._dispatch_home(b.Command())
  self.assertTrue(self.e.preparation_ready);self.assertEqual(self.e.optical_count,1)
 def test_already_homed_xy_does_not_repeat_nested_xy(self):
  self.homed='xy';self.xyz.update(x_ready=1,y_ready=1)
  self.lan_home()
  self.assertEqual(self.nested,[{'Z':''}]);self.assertEqual(self.e.optical_count,1)
 def test_direct_start_rough_and_hot_keep_one_optical(self):
  e=self.e;e.state='printing';e.armed=True
  e.cmd_ROUGH_HOME(b.Command());e.cmd_HOT_PREPARE(b.Command())
  self.assertEqual(e.optical_count,1)
 def test_actual_cx_rough_macro_uses_same_wrapped_g28(self):
  e=self.e
  body=b.compile_method(b.method(b.SRC/'extras/custom_macro.py','CUSTOM_MACRO','cmd_CX_ROUGH_G28'))
  e.objects['prtouch_v2']=types.SimpleNamespace(change_hot_min_temp=lambda temp:None)
  custom=types.SimpleNamespace(printer=e.printer,gcode=e.gcode,default_extruder_temp=220.,default_bed_temp=50.)
  cmd=b.Command(ORIGINAL=1)
  cmd.get_float=lambda name,default=None,**kwargs:float(cmd.get(name,default))
  old=e.gcode.run_script_from_command
  def execute(script):
   if script.startswith('_KAMP_ROUGH_BASE'):body(custom,cmd)
   else:old(script)
  e.gcode.run_script_from_command=execute
  e.cmd_ROUGH_HOME(cmd)
  self.assertTrue(e.preparation_ready);self.assertEqual(e.optical_count,1)
 def test_expired_token_refuses_before_any_home(self):
  e=self.e;e.cmd_LAN_PREPARE(b.Command());self.now+=121
  with self.assertRaisesRegex(ValueError,'expired'):e._dispatch_home(b.Command(Z=''))
  self.assertEqual(e.optical_count,0);self.assertFalse(self.raw);self.assertIsNone(e._lan_home_deadline)
 def test_motor_off_revokes_token(self):
  e=self.e;e.cmd_LAN_PREPARE(b.Command());e._motor_off(0)
  self.assertIsNone(e._lan_home_deadline)
 def test_wait_may_occur_long_after_token_deadline_when_home_completed(self):
  self.lan_home();self.now+=200
  self.e.cmd_LAN_PREPARE(b.Command(WAIT_TEMP=1))
  self.assertTrue(self.e.preparation_ready)
 def test_motor_not_ready_early_return_is_not_a_valid_preparation(self):
  e=self.e;e.home_override.config=types.SimpleNamespace(has_section=lambda name:True,
   getsection=lambda name:types.SimpleNamespace(getint=lambda name:1))
  e.objects['motor_control']=types.SimpleNamespace(is_ready=False)
  e.cmd_LAN_PREPARE(b.Command())
  with self.assertRaisesRegex(ValueError,'single optical baseline'):e._dispatch_home(b.Command(Z=''))
  self.assertEqual(e.optical_count,0);self.assertFalse(e.preparation_ready);self.assertFalse(e.tool.G29_flag)
 def test_lan_token_is_not_consumed_by_outer_x_only_home(self):
  e=self.e;e.cmd_LAN_PREPARE(b.Command())
  # An XY-only override delegates to the underlying XY home in this branch.
  e.home_override.axes='Z'
  e._dispatch_home(b.Command(X=''))
  self.assertIsNotNone(e._lan_home_deadline);self.assertFalse(e.preparation_ready)
  self.assertEqual(e.optical_count,0)
 def test_busy_preparation_cannot_issue_another_token(self):
  for flag in ('_preparing','_hot_running','_optical_running'):
   setattr(self.e,flag,True)
   with self.assertRaises(b.r.RequestRefused):self.e.cmd_LAN_PREPARE(b.Command())
   setattr(self.e,flag,False)
 def test_missing_base_refused_at_wait_before_heating(self):
  with self.assertRaisesRegex(ValueError,'optical baseline'):self.e.cmd_LAN_PREPARE(b.Command(WAIT_TEMP=1))
  self.assertEqual(self.e.optical_count,0)
 def test_prepared_start_without_base_refused_before_arming(self):
  self.e.state='printing'
  with self.assertRaisesRegex(ValueError,'optical baseline'):self.e.cmd_VALIDATE_START(b.Command())
  self.assertFalse(self.e.armed)
 def test_recovery_does_not_consume_lan_token_or_force_calibration(self):
  e=self.e;e.cmd_LAN_PREPARE(b.Command());e.recovering=True
  e._dispatch_home(b.Command(Z=''))
  self.assertFalse(e.preparation_ready);self.assertFalse(e.tool.G29_flag)
  self.assertTrue(e.recovery_optical_done)
 def test_failed_lan_home_clears_flags_and_consumes_token(self):
  e=self.e;e.bad_optical=True;e.cmd_LAN_PREPARE(b.Command())
  with self.assertRaises(ValueError):e._dispatch_home(b.Command(Z=''))
  self.assertFalse(e.tool.G29_flag);self.assertFalse(e.preparation_ready);self.assertIsNone(e._lan_home_deadline)
 def test_busy_request_does_not_mutate_active_job(self):
  e=self.e;e.active=True;e.job_id='keep'
  with self.assertRaises(b.r.RequestRefused):e.cmd_LAN_PREPARE(b.Command())
  self.assertTrue(e.active);self.assertEqual(e.job_id,'keep')
 def test_second_new_job_gets_one_new_optical_only(self):
  self.lan_home();self.assertEqual(self.e.optical_count,1)
  self.lan_home();self.assertEqual(self.e.optical_count,2)
 def test_print_temp_macro_authorizes_before_heat_and_passes_wait_mode(self):
  cfg=configparser.RawConfigParser(inline_comment_prefixes=('#',';'))
  cfg.read(b.PAYLOAD/'mnt/UDISK/printer_data/config/gcode_macro.cfg',encoding='utf-8')
  text=cfg.get('gcode_macro PRINT_TEMP_SET','gcode')
  env=b.Environment(block_start_string='{%',block_end_string='%}',variable_start_string='{',variable_end_string='}')
  for wait in (0,1):
   rendered=env.from_string(text).render(params={'WAIT_TEMP':wait})
   self.assertIn('KAMP_LAN_PREPARE WAIT_TEMP='+str(wait),rendered)
   self.assertLess(rendered.index('KAMP_LAN_PREPARE'),rendered.index('M140'))
if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Cases))
 (b.OUT/'lan-tests.json').write_text(json.dumps({'tests':result.testsRun,'success':result.wasSuccessful(),'scope':'actual saved HomingOverride, real G28 and adjustment methods; simulated hardware and macro body routing'},indent=2))
 raise SystemExit(not result.wasSuccessful())
