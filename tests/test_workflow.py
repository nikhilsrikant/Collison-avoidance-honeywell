import math, sys, types
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import workflow
A = types.SimpleNamespace(m_per_cm=25, ft_per_cm=50, time_scale=4, ta_s=40, ra_s=25, pattern_ta=20, pattern_ra=15,
                          pattern_agl_ft=1000, runway_hdg=None, hmd_ra_m=926, min_descend_ft=300)
KT = 0.514444
def plane(id, role, e, n, alt, psi, kt, vs=0, nose=None):
    return dict(id=id, role=role, label={1:'Our plane',2:'Private jet',4:'Runway'}.get(id), e=e, n=n, alt_ft=alt, psi=psi,
                nose=psi if nose is None else nose, gs_kt=kt, vs_fpm=vs, omega=0, age=0.05)
def run(name, own_alt=1500, jet_alt=1500, jet_vs=0, beacon=None, runway=None, status_fn=None, us=None, jet_n0=3000, own_psi=0):
    adv = workflow.Advisor(A); t = 1000.0; log = []
    V = 120 * KT
    for k in range(400):
        ts = k * 0.1 * 4  # sim seconds
        own = plane(1, 'own', 0, -1500 + V * ts, own_alt, own_psi, 120)
        jet = plane(2, 'other', 30, jet_n0 - V * ts, jet_alt + jet_vs / 60 * ts, 180, 120, jet_vs)
        ac = [own, jet] + ([runway] if runway else [])
        node = {'peers': {}, 'us': us}
        if beacon: node['peers'] = {2: dict(beacon, t=t)}
        if status_fn: node['status'] = status_fn(adv)
        r = adv.step(ac, node, now=t)
        key = (r['level'], r['sense']['vert'], r['sense']['turn'], r['sense']['by'])
        if not log or log[-1][1] != key:
            log.append((round(ts, 1), key, r['tau'], r['say']['text'], r['sense']['why'], r['pattern'], r['intruder'] and r['intruder']['range_src']))
        t += 0.1
    print('==', name)
    for l in log[:6]: print('  ', l)
    return log
run('level head-on, jet not equipped (we are 0 ft apart)')
run('jet CLIMBING per its IMU beacon (camera still level)', beacon=dict(eq=0, pitch=12, roll=0, rssi=-60, type=1, flags=1))
run('jet descending per beacon', beacon=dict(eq=0, pitch=-12, roll=0, rssi=-60, type=1, flags=1))
run('we are 400 ft below, low-ish', own_alt=1100, jet_alt=1500)
run('jet 1500 ft above: no alert expected', jet_alt=3000)
run('traffic pattern (runway marker, 600 ft, aligned)', own_alt=600, jet_alt=600, runway=plane(4,'runway',0,0,0,0,0))
run('equipped peer: node reports tie-break climb', beacon=dict(eq=1, pitch=0, roll=0, rssi=-60, type=1, flags=1),
    status_fn=lambda adv: {'stage': 2 if adv.level == 2 else adv.level, 'vert': 'C' if adv.level == 2 else 'H', 'turn': 'R' if adv.level==2 else 'S', 'neg': 'A' if adv.level==2 else '-', 'ack':0,'ap':0,'auto':0,'comply':1,'peer':2,'peer_eq':1,'ra_by':'L'})
run('ultrasonic sees it (nose sensor 30 cm)', us={'cm': 30, 'age': 0.05}, jet_n0=-1500+30*25+200)
print(workflow.Advisor(A).node_command())
