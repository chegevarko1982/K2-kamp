"""Fault injection against the installer's actual remote transaction function."""
import ast,base64,hashlib,json,pathlib,stat,types,unittest
ROOT=pathlib.Path(__file__).resolve().parent.parent
remote=(ROOT/'tests/fixtures/install_transaction.py').read_text(encoding='utf-8')
node=next(n for n in ast.parse(remote).body if isinstance(n,ast.FunctionDef) and n.name=='commit_payload')

def b64(raw):return base64.b64encode(raw).decode()
class Cases(unittest.TestCase):
    def run_fault(self,fail_call=None,after_replace=True,prefs=False,rollback_failure=False):
        files={'/config':b'old config','/stock.py':b'old stock','/prefs':json.dumps({'power_loss':{'switch':not prefs}}).encode()}
        original=dict(files)
        data={'payload':{'/config':b64(b'new config'),'/stock.py':b64(b'new stock'),'/new.py':b64(b'new module')},
              'manifest':{'files':{p:{'after_sha256':hashlib.sha256(base64.b64decode(v)).hexdigest()} for p,v in {'/config':b64(b'new config'),'/stock.py':b64(b'new stock'),'/new.py':b64(b'new module')}.items()}}}
        calls=[]
        def atomic(path,raw,mode=0o644):
            calls.append(path)
            failure=len(calls)==fail_call or (rollback_failure and len(calls)==fail_call+1)
            if failure and not after_replace:raise OSError('before replace')
            files[path]=raw
            if failure:raise OSError('directory fsync after replace')
        fakeos=types.SimpleNamespace(path=types.SimpleNamespace(exists=lambda p:p in files),
            unlink=lambda p:files.pop(p),sync=lambda:None,stat=lambda p:types.SimpleNamespace(st_mode=0o600))
        ns={'d':data,'atomic':atomic,'base64':base64,'stat':stat,'json':json,'os':fakeos,
            'sha':lambda p:hashlib.sha256(files[p]).hexdigest(),'snapshot':lambda:None}
        exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_remote_install_transaction','exec'),ns)
        originals={p:b64(original[p]) if p in original else None for p in data['payload']}
        error=None
        try:ns['commit_payload'](originals,{p:0o600 for p in originals},json.loads(original['/prefs']),'/prefs')
        except Exception as exc:error=exc
        return files,original,calls,error
    def test_success_writes_all_payload_files(self):
        files,old,calls,error=self.run_fault();self.assertIsNone(error);self.assertIn('/new.py',files)
    def test_fsync_failure_after_replace_restores_that_file_too(self):
        for index in (1,2,3):
            with self.subTest(index=index):
                files,old,calls,error=self.run_fault(index)
                self.assertIsInstance(error,OSError);self.assertEqual(files,old)
    def test_failure_before_new_file_creation_does_not_break_rollback(self):
        files,old,calls,error=self.run_fault(3,after_replace=False)
        self.assertIsInstance(error,OSError);self.assertEqual(files,old)
    def test_preference_replace_fsync_failure_restores_preference(self):
        files,old,calls,error=self.run_fault(4,prefs=True)
        self.assertIsInstance(error,OSError);self.assertEqual(files,old)
    def test_partial_rollback_failure_is_reported_and_other_files_attempted(self):
        files,old,calls,error=self.run_fault(3,rollback_failure=True)
        self.assertIsInstance(error,RuntimeError);self.assertIn('rollback incomplete',str(error))
        self.assertEqual(files['/config'],old['/config']);self.assertEqual(files['/stock.py'],old['/stock.py'])
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Cases))
    (ROOT/'tests/results/installer-tests.json').write_text(json.dumps({'tests':result.testsRun,'success':result.wasSuccessful()},indent=2))
    raise SystemExit(not result.wasSuccessful())
