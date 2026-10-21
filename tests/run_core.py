"""Existing bed-mesh/abort integration tests against candidate adapter core.

New persistence/mechanics are isolated here and covered in test_kamp_v24.py.
This suite executes actual saved bed_mesh/update_config/finalize/profile code.
"""
import importlib.util
import pathlib
import sys
import unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parent.parent
sys.dont_write_bytecode=True
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);return module
recovery=load('extras.kamp_recovery',ROOT/'payload/usr/share/klipper/klippy/extras/kamp_recovery.py')
base=load('base_mesh_tests',ROOT/'tests/legacy_core.py')
recovery=load('extras.kamp_recovery',ROOT/'payload/usr/share/klipper/klippy/extras/kamp_recovery.py')
candidate=load('candidate_adapter',ROOT/'payload/usr/share/klipper/klippy/extras/restore_bed_mesh.py')
base.adapter=candidate
original_fixture=base.fixture
def fixture_with_start_macro(*args,**kwargs):
    result=original_fixture(*args,**kwargs)
    result.printer.objects['gcode_macro START_PRINT']=base.Status(prepare=0)
    return result
base.fixture=fixture_with_start_macro
base.Config.get=lambda self,key,default=None:str(ROOT/'tests/results/core-state') if key=='state_directory' else default
with patch.object(candidate.BedMeshOverride,'_init_recovery',lambda self:None),patch.object(candidate.BedMeshOverride,'_save_checkpoint',lambda self:None):
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(base.AdapterTests))
sys.exit(not result.wasSuccessful())
