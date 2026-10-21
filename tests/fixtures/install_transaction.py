def commit_payload(originals,modes,prefs,prefs_path):
 touched=[];prefs_touched=False
 try:
  for p,encoded in d['payload'].items():
   touched.append(p)  # Include replace followed by a failed directory fsync.
   atomic(p,base64.b64decode(encoded),modes[p])
  if not prefs.get('power_loss',{}).get('switch',False):
   new_prefs=dict(prefs);new_prefs['power_loss']=dict(prefs.get('power_loss',{}),switch=True)
   prefs_touched=True
   atomic(prefs_path,json.dumps(new_prefs).encode(),stat.S_IMODE(os.stat(prefs_path).st_mode))
  for p in d['payload']:
   if sha(p)!=d['manifest']['files'][p]['after_sha256']:raise RuntimeError('Installed hash mismatch: '+p)
  snapshot();os.sync()
 except Exception as error:
  rollback_errors=[]
  for p in reversed(touched):
   try:
    if originals[p] is None:
     if os.path.exists(p):os.unlink(p)
    else:atomic(p,base64.b64decode(originals[p]),modes[p])
   except Exception as rollback_error:rollback_errors.append(p+': '+str(rollback_error))
  if prefs_touched:
   try:atomic(prefs_path,json.dumps(prefs).encode(),stat.S_IMODE(os.stat(prefs_path).st_mode))
   except Exception as rollback_error:rollback_errors.append(prefs_path+': '+str(rollback_error))
  os.sync()
  if rollback_errors:raise RuntimeError('Install failed; rollback incomplete: '+repr(rollback_errors)) from error
  raise
 return touched
