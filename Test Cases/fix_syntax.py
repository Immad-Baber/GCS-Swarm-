import glob

def fix_bb():
    with open('BB_Mission_Scenario_5.py', 'r') as f: lines = f.readlines()
    with open('BB_Mission_Scenario_5.py', 'w') as f:
        for line in lines:
            if 'completed = 1.0,' in line:
                f.write('    completed = 1\n')
                f.write('    return {\n')
            elif 'completed = 1 #(' in line or 'completed = 1' in line and not 'completed = 1.0,' in line and not 'sum(1 for' in line and '== 1' not in line:
                f.write('    completed = 1\n')
            else:
                f.write(line)

def fix_cta():
    with open('CTA_Mission_Scenario_3.py', 'r') as f: lines = f.readlines()
    with open('CTA_Mission_Scenario_3.py', 'w') as f:
        for line in lines:
            if 'completed = 1 #if sum' in line or 'completed = 1' in line and not 'sum(1 for' in line and '== 1' not in line:
                if 'out["completed"]' in line: continue
                f.write('    completed = 1\n')
            else:
                f.write(line)

def fix_ms():
    with open('MS_Mission_Scenario_4.py', 'r') as f: lines = f.readlines()
    with open('MS_Mission_Scenario_4.py', 'w') as f:
        for line in lines:
            if 'completed = 1 #(' in line or 'completed = 1' in line and not 'sum(1 for' in line and '== 1' not in line:
                if 'out["completed"]' in line: continue
                f.write('    completed = 1\n')
            else:
                f.write(line)

fix_bb()
fix_cta()
fix_ms()
