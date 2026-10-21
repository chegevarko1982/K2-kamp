"""Check exact installed bytes and exported fixture provenance, offline."""
import hashlib,json,pathlib
ROOT=pathlib.Path(__file__).resolve().parent.parent
manifest=json.loads((ROOT/'manifest.json').read_text(encoding='utf-8'))
review_manifest=ROOT/'docs/validation/revision-2.4.1-manifest.json'
assert hashlib.sha256(review_manifest.read_bytes()).hexdigest()==manifest['original_review_manifest_sha256'],'reviewed manifest bytes changed'
for path,entry in manifest['files'].items():
    raw=(ROOT/entry['repository_path']).read_bytes()
    assert hashlib.sha256(raw).hexdigest()==entry['sha256'],path
    assert len(raw)==entry['bytes'],path
provenance=json.loads((ROOT/'docs/validation/fixture-provenance.json').read_text(encoding='utf-8'))
for path,expected in provenance['fixtures'].items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==expected,path
print('All 10 payload files and exported source fixtures match SHA-256.')
