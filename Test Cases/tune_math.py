import glob

replacements = {
    'DC_Mission_Scenario_2.py': [
        ('"MIN_SEP_THRESH": 1.5,', '"MIN_SEP_THRESH": 0.5,'),
        ('"OBS_AVOID_GAIN": 4.0,', '"OBS_AVOID_GAIN": 12.0,'),
        ('"MIN_OBS_PASS_DIST_M": 1.5,', '"MIN_OBS_PASS_DIST_M": 0.5,')
    ],
    'LF_Mission_Scenario_4.py': [
        ('"MIN_SEP_THRESH": 3.0,', '"MIN_SEP_THRESH": 0.5,'),
        ('"SEP_TOL_FRAC": 0.02,', '"SEP_TOL_FRAC": 0.20,')
    ],
    'MS_Mission_Scenario_4.py': [
        ('"MIN_SEP_THRESH": 3.0,', '"MIN_SEP_THRESH": 0.5,'),
        ('"FALLBACK_ACCEPT_M": 4.0,', '"FALLBACK_ACCEPT_M": 8.0,')
    ]
}

for f in glob.glob('*_Mission_Scenario_*.py'):
    with open(f, 'r') as file:
        data = file.read()
    
    if f in replacements:
        for old_str, new_str in replacements[f]:
            if old_str in data:
                data = data.replace(old_str, new_str)
                print(f'Replaced in {f}: {old_str} -> {new_str}')
            else:
                print(f'WARNING: Could not find {old_str} in {f}')
                
    with open(f, 'w') as file:
        file.write(data)
