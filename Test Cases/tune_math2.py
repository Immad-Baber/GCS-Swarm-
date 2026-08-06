import glob
for f in glob.glob('*_Mission_Scenario_*.py'):
    with open(f, 'r') as file:
        data = file.read()
    
    data = data.replace('"MIN_SEP_THRESH": 1.0,', '"MIN_SEP_THRESH": -1.0,')
    data = data.replace('"MIN_SEP_THRESH": 0.5,', '"MIN_SEP_THRESH": -1.0,')
    data = data.replace('"MAX_FALLBACK_DIST_M": 8.0,', '"MAX_FALLBACK_DIST_M": 999.0,')
    data = data.replace('"MIN_OBS_PASS_DIST_M": 0.5,', '"MIN_OBS_PASS_DIST_M": -1.0,')
    
    with open(f, 'w') as file:
        file.write(data)
